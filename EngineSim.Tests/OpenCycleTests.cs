using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #27: a cylinder that breathes through its valves, cycle after cycle
public class OpenCycleTests
{
    private const double SparkAdvanceDeg = 20;

    private static readonly CylinderGeometry Geometry = new(95.5, 88.4, 144.2, 10.3);

    [Theory]
    [InlineData(1500)]
    [InlineData(3000)]
    [InlineData(6000)]
    public void SettledCycle_MassInEqualsMassOut(double rpm)
    {
        var result = SettledCycle(rpm);

        // Whatever comes in through the intake has to leave through the exhaust (within 0.5%)
        Assert.Equal(1, result.MassOutG / result.MassInG, 0.005);
    }

    [Theory]
    [InlineData(3000)]
    [InlineData(4500)]
    public void FullThrottleVe_AtMidRpm_IsBetween85And95Percent(double rpm)
    {
        var result = SettledCycle(rpm);

        Assert.InRange(result.VolumetricEfficiency, 0.85, 0.95); // ~94% at 3,000, ~93% at 4,500
    }

    [Fact]
    public void LaterIntakeClosing_LowersVeAtLowRpm()
    {
        // At 1,500 RPM there's time for the rising piston to push charge back out through a late-closing intake valve
        double baseVe = SettledCycle(1500).VolumetricEfficiency;
        double lateVe = SettledCycle(1500, intakeShiftDeg: 20).VolumetricEfficiency;

        Assert.True(lateVe < baseVe, $"late IVC {lateVe:P1} vs base {baseVe:P1}");
    }

    [Fact]
    public void BestIntakeClosing_IsLaterAtHighRpm()
    {
        // The faster the engine, the later the intake should close to finish filling (~555° at 1,500, ~575° at 6,000).
        // Late IVC never wins outright here: that needs intake air momentum, which this model doesn't have.
        Assert.True(BestIntakeShiftDeg(6000) > BestIntakeShiftDeg(1500));
    }

    [Theory]
    [InlineData(1500)]
    [InlineData(3000)]
    [InlineData(6000)]
    public void ResidualGasFraction_IsAFewPercentAtFullLoad(double rpm)
    {
        var result = SettledCycle(rpm);

        Assert.InRange(result.ResidualGasFraction, 0.01, 0.10); // ~4%
    }

    [Theory]
    [InlineData(-30)]
    [InlineData(0)]
    [InlineData(30)]
    public void HeatLoss_StaysARealNumber_AtLowRpmWithTheCamShifted(double intakeShiftDeg)
    {
        var cylinder = OpenCylinder(intakeShiftDeg);

        for (int cycle = 0; cycle < 3; cycle++)
            cylinder.RunOpenCycle(Engine.CrankStepDeg, 1000);

        Assert.False(double.IsNaN(cylinder.HeatLostJ));
        Assert.False(double.IsNaN(cylinder.TempK));
        Assert.True(cylinder.GasMassG > 0);
    }

    [Fact]
    public void SparkOff_BurnsNothing_AndStillBreathes()
    {
        var cylinder = OpenCylinder(sparkAdvanceDeg: null);

        var result = cylinder.RunUntilSettled(Engine.CrankStepDeg, 3000);

        Assert.Equal(0, cylinder.HeatReleasedJ);
        Assert.True(result.IndicatedWorkJ < 0); // pumping air in and out costs work with nothing burning
        Assert.InRange(result.VolumetricEfficiency, 0.85, 1.0);
    }

    [Fact]
    public void SealedCylinder_CantRunAnOpenCycle()
    {
        var sealedCylinder = new Cylinder(Geometry, GasProperties.Air, 540, 101_325, 288.15);

        Assert.Throws<InvalidOperationException>(() => sealedCylinder.RunOpenCycle(Engine.CrankStepDeg, 3000));
    }

    // ---- Helpers ----

    // Full throttle (ambient intake), 20° spark, walls at 450 K, started at firing TDC
    private static Cylinder OpenCylinder(double intakeShiftDeg = 0, double? sparkAdvanceDeg = SparkAdvanceDeg)
    {
        var valvetrain = new Valvetrain();
        valvetrain.Intake.CenterlineDeg += intakeShiftDeg;

        return new Cylinder(Geometry, GasProperties.Air, 0, 101_325, 288.15,
            heatTransfer: new HeatTransfer(Geometry, GasProperties.Air, Engine.WallTempK),
            valvetrain: valvetrain,
            fuel: FuelProperties.PumpGas)
        {
            SparkAdvanceDeg = sparkAdvanceDeg,
        };
    }

    private static CycleResult SettledCycle(double rpm, double intakeShiftDeg = 0) =>
        OpenCylinder(intakeShiftDeg).RunUntilSettled(Engine.CrankStepDeg, rpm);

    // Sweep the intake centerline and return the shift (degrees, negative = earlier) that gives the highest VE
    private static double BestIntakeShiftDeg(double rpm)
    {
        double bestShiftDeg = 0, bestVe = 0;
        for (double shiftDeg = -50; shiftDeg <= 20; shiftDeg += 5)
        {
            double ve = SettledCycle(rpm, shiftDeg).VolumetricEfficiency;
            if (ve > bestVe)
            {
                bestVe = ve;
                bestShiftDeg = shiftDeg;
            }
        }
        return bestShiftDeg;
    }
}
