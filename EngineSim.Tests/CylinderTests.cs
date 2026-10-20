using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #24: one sealed cylinder (valves shut, no combustion) stepped through crank angle
public class CylinderTests
{
    // Start of compression: BDC, standard day (101.325 kPa, 15 °C)
    private const double BdcDeg = 180;
    private const double StartPressurePa = 101_325;
    private const double StartTemperatureK = 288.15;

    private static readonly CylinderGeometry Geometry = new(95.5, 88.4, 144.2, 10.3);

    // Isentropic (no heat, no losses) results: p₁ × CR^γ and T₁ × CR^(γ−1)
    private static readonly double IsentropicPeakPressurePa = StartPressurePa * Math.Pow(10.3, 1.4);
    private static readonly double IsentropicPeakTemperatureK = StartTemperatureK * Math.Pow(10.3, 0.4);

    [Fact]
    public void NewCylinder_TrapsTheAirThatFitsAtBdc()
    {
        var cylinder = NewCylinder();

        Assert.Equal(0.859, cylinder.AirMassG, precision: 3);
        Assert.Equal(StartPressurePa, cylinder.PressurePa, precision: 6);
    }

    [Fact]
    public void CompressionToTdc_MatchesIsentropicPressureAndTemperature()
    {
        var cylinder = NewCylinder();
        StepThrough(cylinder, 180, Engine.CrankStepDeg);

        Assert.Equal(1, cylinder.PressurePa / IsentropicPeakPressurePa, 0.005);       // ~2,653 kPa
        Assert.Equal(1, cylinder.TempK / IsentropicPeakTemperatureK, 0.005);   // ~732 K
    }

    [Fact]
    public void CompressionWork_IsAbout274J()
    {
        var cylinder = NewCylinder();
        StepThrough(cylinder, 180, Engine.CrankStepDeg);

        // WorkJ is work done BY the gas, so compressing makes it negative
        Assert.Equal(1, -cylinder.WorkJ / 273.9, 0.005);
    }

    [Fact]
    public void FullRevolution_GivesAllTheWorkBack()
    {
        var cylinder = NewCylinder();
        StepThrough(cylinder, 180, Engine.CrankStepDeg);
        double compressionWorkJ = -cylinder.WorkJ;

        StepThrough(cylinder, 180, Engine.CrankStepDeg); // TDC → BDC

        Assert.True(Math.Abs(cylinder.WorkJ) < 0.001 * compressionWorkJ);
    }

    [Fact]
    public void HalvingTheStep_BarelyChangesPeakPressure()
    {
        var normal = NewCylinder();
        StepThrough(normal, 180, Engine.CrankStepDeg);

        var fine = NewCylinder();
        StepThrough(fine, 180, Engine.CrankStepDeg / 2);

        Assert.Equal(1, fine.PressurePa / normal.PressurePa, 0.001);
    }

    // ---- Helpers ----

    private static Cylinder NewCylinder() =>
        new(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK);

    // Turn the crank through the given number of degrees in steps of stepDeg
    private static void StepThrough(Cylinder cylinder, double degrees, double stepDeg)
    {
        int steps = (int)Math.Round(degrees / stepDeg);
        for (int i = 0; i < steps; i++)
            cylinder.Step(stepDeg);
    }
}