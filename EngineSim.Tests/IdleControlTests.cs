using EngineSim.Constants;

namespace EngineSim.Tests;

// Ticket #32: idle held by the ECU through the drive-by-wire throttle plate, and stalling for real
public class IdleControlTests
{
    private const double Dt = 0.05;
    private const double TargetIdleRpm = EngineControlUnit.TargetIdleRpm;

    // ---- Holding idle ----

    [Fact]
    public void Idle_SettlesNear700Rpm_WithThePlateAbout1Point5PercentOpen()
    {
        var vehicle = StartedVehicle();
        RunFor(vehicle, seconds: 10);

        var (averageRpm, averageOpening) = AverageOver(vehicle, seconds: 5);

        Assert.Equal(TargetIdleRpm, averageRpm, 30.0);   // ±2% wobble on the target, plus a little lag
        Assert.Equal(0.015, averageOpening, 0.005);       // ~1.45%: torque balances friction + pumping loss
    }

    [Fact]
    public void ThinAir_StillIdlesAt700_WithTheEcuOpeningThePlateMore()
    {
        // 83 kPa is roughly Denver: less air per cycle, so the integral term finds the extra opening by itself
        var seaLevel = StartedVehicle();
        var denver = StartedVehicle(new AmbientConditions { AirPressureKpa = 83 });
        RunFor(seaLevel, seconds: 10);
        RunFor(denver, seconds: 10);

        var (seaLevelRpm, seaLevelOpening) = AverageOver(seaLevel, seconds: 5);
        var (denverRpm, denverOpening) = AverageOver(denver, seconds: 5);

        Assert.Equal(TargetIdleRpm, denverRpm, 30.0);
        Assert.Equal(TargetIdleRpm, seaLevelRpm, 30.0);
        Assert.True(denverOpening > seaLevelOpening, $"Denver {denverOpening:P2} vs sea level {seaLevelOpening:P2}");
    }

    [Fact]
    public void FullThrottleBlip_ReturnsToIdle_WithoutStallingOrDippingBelow600()
    {
        var vehicle = StartedVehicle();
        RunFor(vehicle, seconds: 5);

        vehicle.ThrottleControl(1.0);
        RunFor(vehicle, seconds: 2); // up to the limiter
        vehicle.ThrottleControl(-1.0);

        double lowestRpm = double.MaxValue;
        for (int i = 0; i < 200; i++) // 10 s
        {
            vehicle.Update(Dt);
            lowestRpm = Math.Min(lowestRpm, vehicle.Rpm);
            Assert.True(vehicle.IsRunning, $"stalled at tick {i}");
        }

        Assert.True(lowestRpm >= 600, $"dipped to {lowestRpm:F0} RPM");
        Assert.Equal(TargetIdleRpm, vehicle.Rpm, 30.0);
    }

    // ---- Stalling ----

    [Fact]
    public void WithTheIdleOpeningForcedShut_RpmFallsFarBelowIdle()
    {
        // Engine and ECU driven by hand, with the ECU's idle opening thrown away: the closed-plate leak alone can't hold 700.
        // The engine finds a much lower idle of its own (~350 RPM, where the leak's air just covers friction and pumping)
        // rather than stalling: rough, misfiring combustion at very low RPM isn't modelled. Stalling is tested at the ECU below.
        var engine = NewEngine();
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);
        engine.StartEngine();
        engine.HeldRpm = TargetIdleRpm; // start from a normal idle
        engine.HeldRpm = null;

        for (int i = 0; i < 200; i++) // 10 s
        {
            EcuOutputs outputs = ecu.Update(Dt, SensorsFor(engine));
            engine.SparkAdvanceDeg = outputs.SparkOn ? outputs.SparkAdvanceDeg : null;
            engine.Update(Dt, idleThrottleOpening: 0);
            if (!outputs.AllowRun)
                engine.Stall();
        }

        Assert.True(engine.EngineSpeed < 400, $"held {engine.EngineSpeed:F0} RPM without the ECU's opening");
    }

    [Fact]
    public void Ecu_StallsALowRpmEngine_OnlyAfterTheGracePeriod()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        // 100 RPM, running on its own: fine for the first second after starting, stalled after that.
        // Stop one tick short of 1.0 s and check a few ticks past it, so floating-point sums can't land on the boundary.
        for (int i = 0; i < 19; i++) // 0.95 s
            Assert.True(ecu.Update(Dt, Running(rpm: 100)).AllowRun, $"stalled too early, at {(i + 1) * Dt:F2} s");

        EcuOutputs afterGrace = ecu.Update(Dt, Running(rpm: 100));
        for (int i = 0; i < 2; i++) // to 1.1 s
            afterGrace = ecu.Update(Dt, Running(rpm: 100));
        Assert.False(afterGrace.AllowRun);
        Assert.Equal("engine stalled", afterGrace.Message);
    }

    [Fact]
    public void Ecu_NeverStallsWhileCrankingOrAboveStallRpm()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        for (int i = 0; i < 100; i++) // 5 s each
            Assert.True(ecu.Update(Dt, Running(rpm: 150, cranking: true)).AllowRun);

        var healthy = new EngineControlUnit(FuelProperties.PumpGas);
        for (int i = 0; i < 100; i++)
            Assert.True(healthy.Update(Dt, Running(rpm: 300)).AllowRun);
    }

    // ---- Helpers ----

    private static Engine NewEngine(AmbientConditions? ambient = null) =>
        new(95.5, 88.4, 144.2, 10.3, 6, ambient ?? new AmbientConditions(), FuelProperties.PumpGas);

    // Battery on, start key held for the crank time, then released: the engine is left to idle on its own
    private static Vehicle StartedVehicle(AmbientConditions? ambient = null)
    {
        var vehicle = new Vehicle(NewEngine(ambient));
        vehicle.SwitchBattery();
        vehicle.SetStartKeyHeld(true);
        int crankTicks = (int)Math.Ceiling(Engine.CrankTimeSeconds / Dt) + 1;
        for (int i = 0; i < crankTicks; i++)
            vehicle.Update(Dt);
        vehicle.SetStartKeyHeld(false);

        Assert.True(vehicle.IsRunning);
        return vehicle;
    }

    private static void RunFor(Vehicle vehicle, double seconds)
    {
        int ticks = (int)Math.Round(seconds / Dt);
        for (int i = 0; i < ticks; i++)
            vehicle.Update(Dt);
    }

    // Average RPM and plate opening over a stretch of idling (the target wobbles, so single readings bounce)
    private static (double rpm, double opening) AverageOver(Vehicle vehicle, double seconds)
    {
        int ticks = (int)Math.Round(seconds / Dt);
        double rpmSum = 0, openingSum = 0;
        for (int i = 0; i < ticks; i++)
        {
            vehicle.Update(Dt);
            rpmSum += vehicle.Rpm;
            openingSum += vehicle.ThrottlePlateOpening;
        }
        return (rpmSum / ticks, openingSum / ticks);
    }

    // What the ECU would read from this engine (as Vehicle.ReadSensors does), with a full tank and the battery on
    private static EcuInputs SensorsFor(Engine engine) =>
        new(engine.EngineSpeed, engine.MaxRpm, BatteryConnected: true, FuelLevelPercentage: 100,
            EngineRunning: engine.IsRunning, Cranking: false, engine.MafGPerS, PedalPosition: engine.ThrottlePosition);

    private static EcuInputs Running(double rpm, bool cranking = false) =>
        new(rpm, 6700, BatteryConnected: true, FuelLevelPercentage: 100, EngineRunning: true, Cranking: cranking, MafGPerS: 0, PedalPosition: 0);
}
