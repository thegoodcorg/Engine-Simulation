using EngineSim.Constants;

namespace EngineSim.Tests;

// Ticket #36: friction that rises with RPM
public class FrictionTests
{
    private const double Dt = 0.05;

    [Theory]
    [InlineData(700, 33)]
    [InlineData(3000, 57)]
    [InlineData(6700, 128)]
    public void FrictionTorque_MatchesTheTicketTable(double rpm, double expectedNm)
    {
        // fmepBar = 0.97 + 0.15 × rpmK + 0.05 × rpmK² (rpmK = RPM in thousands), then × 100,000 × displacementM3 / (4 × Math.PI)
        Assert.Equal(expectedNm, NewEngine().FrictionTorqueNm(rpm), 0.5);
    }

    [Fact]
    public void FrictionTorque_AlwaysIncreasesWithRpm()
    {
        var engine = NewEngine();

        double previous = engine.FrictionTorqueNm(0);
        for (double rpm = 100; rpm <= engine.MaxRpm; rpm += 100)
        {
            double friction = engine.FrictionTorqueNm(rpm);
            Assert.True(friction > previous, $"friction didn't rise at {rpm} RPM");
            previous = friction;
        }
    }

    [Fact]
    public void StoppedEngine_HasNoFriction()
    {
        // The base 0.97 bar only applies while the crank turns: a stopped engine isn't slowed, and doesn't spin backwards
        var engine = NewEngine();

        engine.Update(Dt, sparkOn: false, idleFloorRpm: 0, fuelPerCycleG: 0);

        Assert.Equal(0, engine.NetTorque);
        Assert.Equal(0, engine.EngineSpeed);
    }

    [Theory]
    [InlineData(0.1)]
    [InlineData(0.2)]
    public void FreeRevvingAtPartThrottle_SettlesBelowRedline(double throttle)
    {
        // Friction rises with RPM, and the throttle's air supply stops rising (#35), so the engine finds a speed where
        // they balance instead of racing to the limiter (~3,150 RPM at 10%, ~5,350 RPM at 20%)
        var engine = NewEngine();
        engine.StartEngine();
        engine.ThrottlePosition = throttle;

        RunFor(engine, seconds: 10);
        double rpmBefore = engine.EngineSpeed;
        RunFor(engine, seconds: 2);

        Assert.True(engine.EngineSpeed < engine.MaxRpm - 500, $"ran up to {engine.EngineSpeed:F0} RPM");
        Assert.Equal(rpmBefore, engine.EngineSpeed, 5); // steady to within 5 RPM over the last 2 seconds
    }

    // ---- Helpers ----

    private static Engine NewEngine() =>
        new(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas);

    // Run with spark on and the stoichiometric fuel for the air the engine is taking in, idle floor at 700 RPM
    private static void RunFor(Engine engine, double seconds)
    {
        int ticks = (int)Math.Round(seconds / Dt);
        for (int i = 0; i < ticks; i++)
            engine.Update(Dt, sparkOn: true, idleFloorRpm: 700, fuelPerCycleG: engine.AirMassPerCycleG / FuelProperties.PumpGas.StoichiometricAfr);
    }
}
