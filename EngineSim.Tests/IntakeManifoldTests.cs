using EngineSim.Constants;
using EngineSim.VehicleSystems.Air;

namespace EngineSim.Tests;

// Ticket #35: throttle body and intake manifold (plenum). MAF from throttle flow, MAP from the plenum.
// The flow equation itself (no flow at equal pressures, choked flow constant) is tested in GasHelperTests.
public class IntakeManifoldTests
{
    private const double Tick = 0.05;               // one Vehicle tick
    private const double DisplacementLiters = 3.799;
    private const double OutsidePressureKpa = 101.325;

    // ---- Throttle area ----

    [Fact]
    public void ThrottleArea_IsTheBypassAloneAtPedal0()
    {
        var manifold = NewManifold();

        Assert.Equal(21.7 / 1_000_000, manifold.ThrottleAreaM2(0), precision: 12);
    }

    [Fact]
    public void ThrottleArea_IsTheFullBorePlusTheBypassAtPedal1()
    {
        var manifold = NewManifold();
        double fullBoreM2 = Math.PI / 4 * 70 * 70 / 1_000_000; // 70 mm throttle bore

        Assert.Equal(fullBoreM2 + 21.7 / 1_000_000, manifold.ThrottleAreaM2(1), precision: 12);
    }

    [Fact]
    public void ThrottleArea_GrowsAsThePedalGoesDown()
    {
        var manifold = NewManifold();

        double previous = 0;
        for (double pedal = 0; pedal <= 1.0001; pedal += 0.05)
        {
            double area = manifold.ThrottleAreaM2(pedal);
            Assert.True(area >= previous, $"area shrank at pedal {pedal:F2}");
            previous = area;
        }
    }

    // ---- Plenum pressure ----

    [Fact]
    public void Plenum_StartsAtOutsidePressure()
    {
        Assert.Equal(OutsidePressureKpa, NewManifold().MapKpa, precision: 6);
    }

    [Fact]
    public void Plenum_WithTheEngineStopped_FillsBackToOutsidePressure()
    {
        // Pull the plenum down with the engine turning at closed throttle, then stop the engine
        var manifold = NewManifold();
        RunFor(manifold, seconds: 1, rpm: 3000, pedal: 0);
        Assert.True(manifold.MapKpa < 30);

        RunFor(manifold, seconds: 2, rpm: 0, pedal: 0);

        Assert.Equal(OutsidePressureKpa, manifold.MapKpa, 0.01);
    }

    [Theory]
    [InlineData(700, 0)]
    [InlineData(2000, 0.1)]
    [InlineData(3000, 0.3)]
    [InlineData(3000, 1)]
    [InlineData(6000, 1)]
    public void SteadyState_ThrottleFlowEqualsEngineFlow(double rpm, double pedal)
    {
        var manifold = Settled(rpm, pedal);

        Assert.Equal(1, manifold.ThrottleFlowKgPerS / manifold.EngineFlowKgPerS, 0.01);
    }

    [Theory]
    [InlineData(1000)]
    [InlineData(3000)]
    [InlineData(6700)]
    public void FullThrottle_PlenumHoldsSteady(double rpm)
    {
        // Wide open, the throttle fills the plenum in well under a millisecond. The implicit step must not overshoot
        // and flip back and forth from one substep to the next.
        var manifold = Settled(rpm, 1);
        double before = manifold.MapKpa;

        for (int i = 0; i < 6; i++)
        {
            manifold.Update(0.001, rpm, 1);
            Assert.Equal(before, manifold.MapKpa, 0.01);
        }
    }

    // ---- Expected numbers ----

    [Fact]
    public void Idle_MapAndMaf_MatchTheTicket()
    {
        var manifold = Settled(700, 0);

        Assert.Equal(17.4, manifold.MapKpa, 0.5);   // ≈ 17 kPa
        Assert.Equal(4.2, manifold.MafGPerS, 0.1);  // ≈ 4.2 g/s
    }

    [Fact]
    public void FullThrottleAtRedline_MapAndMaf_MatchTheTicket()
    {
        var manifold = Settled(6700, 1);

        Assert.Equal(99.0, manifold.MapKpa, 0.5);     // ≈ 99 kPa: barely below outside pressure
        Assert.Equal(228.6, manifold.MafGPerS, 1.0);  // ≈ 229 g/s
    }

    [Fact]
    public void SmallThrottle_IsChoked_SoMafBarelyChangesWithRpm()
    {
        // At 5% pedal the gap is tiny and the air through it is at the speed of sound: the engine can pull as hard as it likes
        double mafAt2000 = Settled(2000, 0.05).MafGPerS;
        double mafAt5000 = Settled(5000, 0.05).MafGPerS;

        Assert.Equal(1, mafAt5000 / mafAt2000, 0.05);
    }

    [Fact]
    public void FullThrottle_MafRoughlyDoublesFrom3000To6000()
    {
        // Wide open the throttle isn't the limit, so twice the RPM pulls about twice the air (~104 → ~206 g/s)
        double mafAt3000 = Settled(3000, 1).MafGPerS;
        double mafAt6000 = Settled(6000, 1).MafGPerS;

        Assert.InRange(mafAt6000 / mafAt3000, 1.8, 2.1);
    }

    [Fact]
    public void TipIn_MafJumpsBeforeTheManifoldPressureCatchesUp()
    {
        // Settle at 10% pedal, then floor it: the plenum is still at low pressure, so air rushes in through the
        // suddenly huge throttle faster than the engine takes it. That's the tip-in spike an OBD2 reader shows.
        var manifold = Settled(3000, 0.1);
        double steadyFullThrottle = Settled(3000, 1).MafGPerS;
        double steadyFullThrottleMap = Settled(3000, 1).MapKpa;

        manifold.Update(0.001, 3000, 1);

        Assert.True(manifold.MafGPerS > steadyFullThrottle, $"tip-in MAF {manifold.MafGPerS:F0} vs steady {steadyFullThrottle:F0} g/s");
        Assert.True(manifold.ThrottleFlowKgPerS > manifold.EngineFlowKgPerS);
        Assert.True(manifold.MapKpa < steadyFullThrottleMap);
    }

    [Theory]
    [InlineData(700, 0)]
    [InlineData(3000, 0.2)]
    [InlineData(6000, 1)]
    public void HalvingTheSubstep_BarelyChangesSteadyMap(double rpm, double pedal)
    {
        var normal = Settled(rpm, pedal);
        var fine = Settled(rpm, pedal, subStepSeconds: IntakeManifold.DefaultSubStepSeconds / 2);

        Assert.Equal(1, fine.MapKpa / normal.MapKpa, 0.005);
    }

    [Fact]
    public void ThinAir_FullThrottleMapIsAbout18PercentLower()
    {
        // 83 kPa is roughly Denver: 83 / 101.325 = 0.819
        var seaLevel = Settled(3000, 1);
        var denver = Settled(3000, 1, new AmbientConditions { AirPressureKpa = 83 });

        Assert.Equal(83 / OutsidePressureKpa, denver.MapKpa / seaLevel.MapKpa, 0.01);
        Assert.Equal(83 / OutsidePressureKpa, denver.MafGPerS / seaLevel.MafGPerS, 0.01);
    }

    // ---- In the engine ----

    [Fact]
    public void ClosedThrottleAt4000_BrakesHarderThanFrictionAlone()
    {
        // Spark off holds the engine at the 4,000 RPM floor with no combustion (like a fuel cut on overrun).
        // Pulling air in against the near-vacuum plenum (pumping loss) adds to friction: that's engine braking.
        var engine = RunningEngineHeldAt(4000, throttle: 0);

        Assert.True(engine.NetTorque < -engine.FrictionTorque,
            $"net torque {engine.NetTorque:F1} Nm vs friction {engine.FrictionTorque:F1} Nm");
        Assert.True(engine.ManifoldPressureKpa < 20);
    }

    [Fact]
    public void PartThrottle_GivesLessAirPerCycleThanFullThrottle()
    {
        var part = RunningEngineHeldAt(3000, throttle: 0.1);
        var full = RunningEngineHeldAt(3000, throttle: 1);

        Assert.True(part.ManifoldPressureKpa < full.ManifoldPressureKpa);
        Assert.True(part.AirMassPerCycleG < full.AirMassPerCycleG);
        Assert.Equal(part.MafGPerS / full.MafGPerS, part.AirMassPerCycleG / full.AirMassPerCycleG, 0.01); // at a fixed RPM, MAF follows air per cycle
    }

    // ---- Helpers ----

    private static IntakeManifold NewManifold(AmbientConditions? ambient = null, double subStepSeconds = IntakeManifold.DefaultSubStepSeconds) =>
        new(ambient ?? new AmbientConditions(), DisplacementLiters, subStepSeconds);

    private static void RunFor(IntakeManifold manifold, double seconds, double rpm, double pedal)
    {
        int ticks = (int)Math.Round(seconds / Tick);
        for (int i = 0; i < ticks; i++)
            manifold.Update(Tick, rpm, pedal);
    }

    // A manifold run for 3 seconds at a fixed RPM and pedal: long enough for the plenum to settle
    private static IntakeManifold Settled(double rpm, double pedal, AmbientConditions? ambient = null, double subStepSeconds = IntakeManifold.DefaultSubStepSeconds)
    {
        var manifold = NewManifold(ambient, subStepSeconds);
        RunFor(manifold, 3, rpm, pedal);
        return manifold;
    }

    // An engine held at a fixed RPM: spark off means no combustion, so friction and pumping slow it down,
    // and the idle floor catches it at exactly floorRpm every tick. 2 seconds lets the plenum settle.
    private static Engine RunningEngineHeldAt(double floorRpm, double throttle)
    {
        var engine = new Engine(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas);
        engine.StartEngine();
        engine.ThrottlePosition = throttle;

        for (int i = 0; i < 40; i++)
            engine.Update(Tick, sparkOn: false, idleFloorRpm: floorRpm, fuelPerCycleG: 0);

        return engine;
    }
}
