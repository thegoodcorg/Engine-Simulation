using EngineSim;
using EngineSim.Constants;
using EngineSim.VehicleSystems.Electrical;

namespace EngineSim.Tests;

public class IgnitionTests
{
    private const double Dt = 0.05;

    [Fact]
    public void IgnitionSystem_StartsWithSparkOff()
    {
        Assert.False(new IgnitionSystem().SparkOn);
    }

    [Fact]
    public void IgnitionSystem_SetSpark_TurnsSparkOnAndOff()
    {
        var ignition = new IgnitionSystem();

        ignition.SetSpark(true);
        Assert.True(ignition.SparkOn);

        ignition.SetSpark(false);
        Assert.False(ignition.SparkOn);
    }

    [Fact]
    public void Engine_NoSpark_MakesNoTorqueAndSlowsDown()
    {
        var engine = new Engine(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas);
        engine.StartEngine();
        engine.ThrottlePosition = 1;
        engine.SparkAdvanceDeg = 20;
        engine.HeldRpm = 3000; // get it running at speed, like on a dyno, then let it go
        for (int i = 0; i < 10; i++)
            engine.Update(Dt, idleThrottleOpening: 0);
        engine.HeldRpm = null;

        // Spark off. A cylinder that trapped its charge before the cut still burns it, so give it two ticks to clear
        engine.SparkAdvanceDeg = null;
        engine.Update(Dt, idleThrottleOpening: 0);
        double rpmBefore = engine.EngineSpeed;
        engine.Update(Dt, idleThrottleOpening: 0);

        // Nothing burning: the gas gives back about what it took (~0 Nm at full throttle), against hundreds with spark on
        Assert.InRange(engine.EngineTorqueNm, -50, 5);
        Assert.Equal(0, engine.FuelBurnedG);
        Assert.True(engine.EngineSpeed < rpmBefore);
    }

    // Ticket #6 "done when": holding full throttle bounces off the limiter instead of sitting at redline
    [Fact]
    public void Vehicle_FullThrottle_BouncesOffRevLimiter()
    {
        var vehicle = StartedVehicle();
        vehicle.ThrottleControl(1);

        var rpms = new List<double>();
        var sparks = new List<bool>();
        for (int i = 0; i < 60; i++) // 3 seconds: reaches redline, then bounces
        {
            vehicle.Update(Dt);
            rpms.Add(vehicle.Rpm);
            sparks.Add(vehicle.SparkOn);
        }

        // Look at the last second, well after reaching redline
        var settledRpms = rpms.Skip(40).ToList();
        var settledSparks = sparks.Skip(40).ToList();

        Assert.All(settledRpms, rpm => Assert.True(rpm <= vehicle.MaxRpm));
        Assert.Contains(true, settledSparks);                         // spark comes back...
        Assert.Contains(false, settledSparks);                        // ...and gets cut again
        Assert.Contains(settledRpms, rpm => rpm < vehicle.MaxRpm - 100); // RPM really drops, not pinned at redline
    }

    [Fact]
    public void Vehicle_BatteryCut_TurnsSparkOffAndStalls()
    {
        var vehicle = StartedVehicle();
        Assert.True(vehicle.SparkOn);

        vehicle.SwitchBattery();
        vehicle.Update(Dt);

        Assert.False(vehicle.SparkOn);
        Assert.False(vehicle.IsRunning);
    }

    // Battery on, start key held for the crank, then released
    private static Vehicle StartedVehicle()
    {
        var vehicle = new Vehicle(new Engine(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas));
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
