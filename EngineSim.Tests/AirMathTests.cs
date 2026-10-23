using EngineSim;
using EngineSim.Constants;

namespace EngineSim.Tests;

// Ticket #5's fuel plumbing: the engine reports the fuel it used, and the Vehicle drains it from the tank.
// #5's torque math (air × fuel energy × a fixed efficiency) was replaced by the crank-angle model in #29;
// torque and fuel per cycle are tested in CrankAngleEngineTests now.
public class AirMathTests
{
    private const double Dt = 0.05;
    private static readonly FuelProperties Fuel = FuelProperties.PumpGas;

    [Fact]
    public void EngineOff_MakesNoTorqueAndUsesNoFuel()
    {
        // Spark is set, but the crank never turns, so no cylinder ever traps air or gets fuel
        var engine = NewEngine();
        engine.ThrottlePosition = 1;
        engine.SparkAdvanceDeg = 20;

        engine.Update(Dt, idleThrottleOpening: 0);

        Assert.Equal(0, engine.FuelBurnedG);
        Assert.Equal(0, engine.EngineTorqueNm);
    }

    // ---- The whole car: the engine fuels each cylinder for the air it traps, Vehicle drains the tank ----

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
        var vehicle = new Vehicle(NewEngine());
        vehicle.SwitchBattery();

        for (int i = 0; i < 20; i++) // 1 second, never started
            vehicle.Update(Dt);

        Assert.Equal(80, vehicle.FuelLevelL);
    }

    // ---- Helpers ----

    private static Engine NewEngine() => new(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), Fuel);

    // Battery on, start key held for the crank, then released
    private static Vehicle StartedVehicle()
    {
        var vehicle = new Vehicle(NewEngine());
        vehicle.SwitchBattery();
        vehicle.SetStartKeyHeld(true);
        // A whole number of ticks past the crank time (comparing float times can land either side)
        int crankTicks = (int)Math.Ceiling(Engine.CrankTimeSeconds / Dt) + 1;
        for (int i = 0; i < crankTicks; i++)
            vehicle.Update(Dt);
        vehicle.SetStartKeyHeld(false);
        vehicle.Update(Dt);

        Assert.True(vehicle.IsRunning);
        return vehicle;
    }
}
