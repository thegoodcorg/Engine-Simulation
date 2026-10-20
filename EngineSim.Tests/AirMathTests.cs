using EngineSim;
using EngineSim.Constants;

namespace EngineSim.Tests;

// Ticket #5: torque from the air the engine takes in, and the fuel that air burns
public class AirMathTests
{
    private const double Dt = 0.05;
    private static readonly FuelProperties Fuel = FuelProperties.PumpGas;

    // Hand calculation for the default engine (95.5 × 88.4 mm, 6 cyl) on a standard day, full throttle:
    //   air  = 0.9 × 1.2250 kg/m³ × 0.0037993 m³ = 0.0041887 kg = 4.1887 g
    //   fuel = air / 14.7                         = 0.28495 g
    //   work = fuel (kg) × 44,000,000 J/kg × 0.36 = 4,513.6 J
    //   torque = work / 4π                        = 359.18 Nm
    private const double FullThrottleAirG = 4.1887;
    private const double FullThrottleTorqueNm = 359.18;

    // ---- Torque ----

    [Fact]
    public void AirPerCycle_FullThrottle_MatchesHandCalculation()
    {
        var engine = NewEngine();
        engine.ThrottlePosition = 1;

        Assert.Equal(FullThrottleAirG, engine.AirMassPerCycleG, 1e-3);
    }

    [Fact]
    public void Torque_FullThrottleStandardDay_MatchesHandCalculation()
    {
        var engine = RunningEngine(throttle: 1);

        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(engine));

        Assert.Equal(FullThrottleTorqueNm, engine.EngineTorqueNm, 0.01);
        Assert.Equal(FullThrottleTorqueNm, engine.MaxTorqueNm, 0.01);
    }

    [Fact]
    public void Torque_ThinAir_IsAbout18PercentLess()
    {
        // 83 kPa is roughly Denver. Density, and so torque, scales with pressure: 83 / 101.325 = 0.819
        var thinAir = new AmbientConditions { AirPressureKpa = 83 };
        var engine = new Engine(95.5, 88.4, 6, thinAir, Fuel);

        Assert.Equal(FullThrottleTorqueNm * 83 / 101.325, engine.MaxTorqueNm, 0.01); // ~294.2 Nm
    }

    [Fact]
    public void Torque_ScalesWithThrottle()
    {
        var engine = RunningEngine(throttle: 0.5);

        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(engine));

        Assert.Equal(FullThrottleTorqueNm / 2, engine.EngineTorqueNm, 0.01);
    }

    [Fact]
    public void RichMix_UsesMoreFuelButMakesTheSameTorque()
    {
        var stoich = RunningEngine(throttle: 1);
        var rich = RunningEngine(throttle: 1);
        stoich.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(stoich)); // get both cranks turning
        rich.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(rich));

        stoich.Update(Dt, sparkOn: true, idleFloorRpm: 0, stoich.AirMassPerCycleG / 14.7);
        rich.Update(Dt, sparkOn: true, idleFloorRpm: 0, rich.AirMassPerCycleG / 12.5);

        // The extra fuel has no air to burn with, so it adds no torque, but it's still used up
        Assert.Equal(stoich.EngineTorqueNm, rich.EngineTorqueNm, 1e-9);
        Assert.True(rich.FuelBurnedG > stoich.FuelBurnedG);
    }

    // ---- Airflow (what the MAF reads) ----

    [Fact]
    public void Airflow_IsAirPerCycleTimesCyclesPerSecond()
    {
        var engine = RunningEngine(throttle: 1);
        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(engine)); // get the crank turning

        Assert.True(engine.EngineSpeed > 0);
        Assert.Equal(engine.AirMassPerCycleG * engine.EngineSpeed / 120, engine.AirflowGPerS, 1e-9);
    }

    // ---- Fuel used ----

    [Fact]
    public void FuelUsed_OneMinuteAt3000Rpm_MatchesHandCalculation()
    {
        // 3,000 RPM = 25 cycles/s. 0.28495 g × 25 × 60 s = 427.4 g = 0.4274 kg = 0.574 L
        double fuelPerCycleG = FullThrottleAirG / 14.7;

        double grams = Engine.FuelUsedG(fuelPerCycleG, rpm: 3000, dt: 60);

        Assert.Equal(0.574, grams / 1000 / Fuel.DensityKgPerL, 0.001);
    }

    [Fact]
    public void FuelUsed_ManySmallTicks_AddUpToOneBigOne()
    {
        double fuelPerCycleG = FullThrottleAirG / 14.7;
        double total = 0;
        for (int i = 0; i < 1200; i++) // 60 s of 0.05 s ticks
            total += Engine.FuelUsedG(fuelPerCycleG, rpm: 3000, Dt);

        Assert.Equal(Engine.FuelUsedG(fuelPerCycleG, rpm: 3000, dt: 60), total, 1e-6);
    }

    [Fact]
    public void Update_FuelBurned_UsesTheRpmAtTheStartOfTheTick()
    {
        var engine = RunningEngine(throttle: 1);
        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(engine)); // get the crank turning
        double rpm = engine.EngineSpeed;
        double fuelPerCycleG = StoichFuel(engine);

        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, fuelPerCycleG);

        Assert.Equal(Engine.FuelUsedG(fuelPerCycleG, rpm, Dt), engine.FuelBurnedG, 1e-9);
    }

    [Fact]
    public void NoSpark_BurnsNoFuel()
    {
        var engine = RunningEngine(throttle: 1);
        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(engine)); // get the crank turning

        engine.Update(Dt, sparkOn: false, idleFloorRpm: 0, StoichFuel(engine));

        Assert.Equal(0, engine.FuelBurnedG);
    }

    [Fact]
    public void EngineOff_BurnsNoFuel()
    {
        var engine = NewEngine(); // never started
        engine.ThrottlePosition = 1;

        engine.Update(Dt, sparkOn: true, idleFloorRpm: 0, StoichFuel(engine));

        Assert.Equal(0, engine.FuelBurnedG);
        Assert.Equal(0, engine.EngineTorqueNm);
    }

    // ---- The whole car: ECU decides fuel from the MAF reading, Vehicle drains the tank ----

    [Fact]
    public void Vehicle_Running_DrainsTheTank()
    {
        var vehicle = StartedVehicle();
        vehicle.ThrottleControl(1);
        double before = vehicle.FuelLevelL;

        for (int i = 0; i < 20; i++) // 1 second
            vehicle.Update(Dt);

        Assert.True(vehicle.FuelLevelL < before);
    }

    [Fact]
    public void Vehicle_EngineOff_DoesNotDrainTheTank()
    {
        var vehicle = new Vehicle(95.5, 88.4, 6, new AmbientConditions());
        vehicle.SwitchBattery();

        for (int i = 0; i < 20; i++) // 1 second, never started
            vehicle.Update(Dt);

        Assert.Equal(80, vehicle.FuelLevelL);
    }

    // ---- Helpers ----

    private static Engine NewEngine() => new(95.5, 88.4, 6, new AmbientConditions(), Fuel);

    private static Engine RunningEngine(double throttle)
    {
        var engine = NewEngine();
        engine.StartEngine();
        engine.ThrottlePosition = throttle;
        return engine;
    }

    // The fuel the ECU would ask for: the engine's air at the stoichiometric ratio
    private static double StoichFuel(Engine engine) => engine.AirMassPerCycleG / Fuel.StoichiometricAfr;

    // Battery on, start key held for the crank, then released
    private static Vehicle StartedVehicle()
    {
        var vehicle = new Vehicle(95.5, 88.4, 6, new AmbientConditions());
        vehicle.SwitchBattery();
        vehicle.SetStartKeyHeld(true);
        for (int i = 0; i * Dt <= Engine.CrankTimeSeconds + Dt; i++)
            vehicle.Update(Dt);
        vehicle.SetStartKeyHeld(false);
        vehicle.Update(Dt);

        Assert.True(vehicle.IsRunning);
        return vehicle;
    }
}
