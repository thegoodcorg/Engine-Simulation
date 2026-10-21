using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #27: cam lift, valve flow area and the two-way flow equation
public class ValvetrainTests
{
    private const double Area = 0.001; // m², for the flow equation tests
    private static readonly GasProperties Air = GasProperties.Air;

    // ---- Cam lift ----

    [Fact]
    public void DefaultCams_GiveTheTicketsValveEvents()
    {
        var valvetrain = new Valvetrain();

        Assert.Equal(350, valvetrain.Intake.OpensDeg);   // IVO: 10° before overlap TDC
        Assert.Equal(590, valvetrain.Intake.ClosesDeg);  // IVC: 50° after BDC
        Assert.Equal(130, valvetrain.Exhaust.OpensDeg);  // EVO: 50° before BDC
        Assert.Equal(370, valvetrain.Exhaust.ClosesDeg); // EVC: 10° after overlap TDC
    }

    [Theory]
    [InlineData(0)]
    [InlineData(300)]
    [InlineData(349)]
    [InlineData(600)]
    [InlineData(700)]
    public void Lift_IsZeroOutsideTheOpenWindow(double crankAngleDeg)
    {
        var intake = new Valvetrain().Intake; // open 350°–590°

        Assert.Equal(0, intake.LiftMm(crankAngleDeg));
    }

    [Fact]
    public void Lift_IsZeroAtBothEndsAndMaxAtTheCenterline()
    {
        var intake = new Valvetrain().Intake;

        Assert.Equal(0, intake.LiftMm(intake.OpensDeg), precision: 9);
        Assert.Equal(intake.MaxLiftMm, intake.LiftMm(intake.CenterlineDeg), precision: 9);
        Assert.Equal(0, intake.LiftMm(intake.ClosesDeg), precision: 9);
    }

    [Fact]
    public void Lift_IsHalfwayAQuarterOfTheWayThrough()
    {
        // A quarter of the way through the window is halfway up the cosine hump
        var intake = new Valvetrain().Intake;

        Assert.Equal(intake.MaxLiftMm / 2, intake.LiftMm(intake.OpensDeg + intake.DurationDeg / 4), precision: 9);
    }

    [Theory]
    [InlineData(400)]
    [InlineData(470)]
    [InlineData(550)]
    public void Lift_RepeatsEvery720Degrees(double crankAngleDeg)
    {
        // The crank angle keeps growing as cycles repeat, so the cam has to fold it back into one cycle
        var intake = new Valvetrain().Intake;

        Assert.Equal(intake.LiftMm(crankAngleDeg), intake.LiftMm(crankAngleDeg + 720), precision: 9);
        Assert.Equal(intake.LiftMm(crankAngleDeg), intake.LiftMm(crankAngleDeg + 1440), precision: 9);
    }

    [Fact]
    public void MovingTheCenterline_MovesTheWholeLiftCurve()
    {
        var intake = new Valvetrain().Intake;
        double liftBefore = intake.LiftMm(470);

        intake.CenterlineDeg -= 20; // 20° of cam advance: everything happens 20° earlier

        Assert.Equal(330, intake.OpensDeg);
        Assert.Equal(570, intake.ClosesDeg);
        Assert.Equal(liftBefore, intake.LiftMm(450), precision: 9);
    }

    // ---- Flow area ----

    [Fact]
    public void FlowArea_IsZeroWhenTheValveIsShut()
    {
        var intake = new Valvetrain().Intake;

        Assert.Equal(0, intake.FlowAreaM2(100));
    }

    [Fact]
    public void FlowArea_AtLowLift_IsTheCurtainArea()
    {
        // Quarter of the way through: 5 mm of lift. Curtain = Math.PI × 37 × 5 = 581 mm² per valve, under the port's 871 mm²
        var intake = new Valvetrain().Intake;
        double angle = intake.OpensDeg + intake.DurationDeg / 4;

        double expectedM2 = Math.PI * intake.ValveDiameterMm * 5 * intake.ValvesPerCylinder / 1_000_000;

        Assert.Equal(expectedM2, intake.FlowAreaM2(angle), precision: 9);
    }

    [Fact]
    public void FlowArea_AtFullLift_IsCappedAtThePortArea()
    {
        // 10 mm of lift: curtain = 1,162 mm² per valve, but the port only has 871 mm², so the port limits it
        var intake = new Valvetrain().Intake;

        double portAreaM2 = Math.PI / 4 * intake.PortDiameterMm * intake.PortDiameterMm * intake.ValvesPerCylinder / 1_000_000;

        Assert.Equal(portAreaM2, intake.FlowAreaM2(intake.CenterlineDeg), precision: 9);
    }

    // ---- Flow equation ----

    [Fact]
    public void Flow_IsZeroAtEqualPressures()
    {
        Assert.Equal(0, Valvetrain.MassFlowKgPerS(Area, 101_325, 288.15, 101_325, 288.15, Air));
    }

    [Fact]
    public void Flow_MatchesAHandCalculation()
    {
        // 101,325 → 90,000 Pa: pressure ratio 0.888, flow factor 0.4436
        // 0.7 × 0.001 × 101,325 / Math.Sqrt(287.05 × 288.15) × 0.4436 = 0.1094 kg/s
        Assert.Equal(0.1094, Valvetrain.MassFlowKgPerS(Area, 101_325, 288.15, 90_000, 288.15, Air), precision: 4);
    }

    [Fact]
    public void SwappingThePressures_ReversesTheFlow()
    {
        double forward = Valvetrain.MassFlowKgPerS(Area, 101_325, 288.15, 90_000, 288.15, Air);
        double backward = Valvetrain.MassFlowKgPerS(Area, 90_000, 288.15, 101_325, 288.15, Air);

        Assert.True(forward > 0);
        Assert.Equal(-forward, backward, precision: 12);
    }

    [Fact]
    public void ChokedFlow_StaysConstantBelowTheCriticalRatio()
    {
        // Below a ratio of 0.528 the gas in the hole is at the speed of sound: lowering the downstream pressure adds nothing
        double atHalf = Valvetrain.MassFlowKgPerS(Area, 200_000, 300, 100_000, 300, Air);    // ratio 0.5
        double atQuarter = Valvetrain.MassFlowKgPerS(Area, 200_000, 300, 50_000, 300, Air);  // ratio 0.25
        double nearVacuum = Valvetrain.MassFlowKgPerS(Area, 200_000, 300, 1_000, 300, Air);  // ratio 0.005

        Assert.Equal(0.3267, atHalf, precision: 4);
        Assert.Equal(atHalf, atQuarter, precision: 12);
        Assert.Equal(atHalf, nearVacuum, precision: 12);
    }

    [Fact]
    public void Flow_RisesSmoothlyIntoChoking()
    {
        // The two flow formulas meet at the critical ratio: just above and just below give the same flow
        double critical = Math.Pow(2 / 2.4, 1.4 / 0.4); // 0.528 for air
        double justAbove = Valvetrain.MassFlowKgPerS(Area, 200_000, 300, 200_000 * (critical + 1e-6), 300, Air);
        double justBelow = Valvetrain.MassFlowKgPerS(Area, 200_000, 300, 200_000 * (critical - 1e-6), 300, Air);

        Assert.Equal(justBelow, justAbove, precision: 6);
    }

    [Fact]
    public void HotterUpstreamGas_FlowsLess()
    {
        // Hot gas is thinner, so fewer kg get through the same hole: flow scales with 1 / Math.Sqrt(upstream temperature)
        double cool = Valvetrain.MassFlowKgPerS(Area, 101_325, 288.15, 90_000, 288.15, Air);
        double hot = Valvetrain.MassFlowKgPerS(Area, 101_325, 1000, 90_000, 288.15, Air);

        Assert.Equal(cool * Math.Sqrt(288.15 / 1000), hot, precision: 9);
    }
}
