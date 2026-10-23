using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #26: heat lost from the gas to the cylinder walls (Woschni)
public class HeatTransferTests
{
    // Cycle in the epic's convention: 0° / 720° = firing TDC, so compression starts at BDC = 540°
    private const double BdcDeg = 540;
    private const double StartPressurePa = 101_325;
    private const double StartTemperatureK = 288.15;

    private static readonly CylinderGeometry Geometry = new(95.5, 88.4, 144.2, 10.3);
    private static readonly FuelProperties Fuel = FuelProperties.PumpGas;

    [Fact]
    public void FullLoadAt3000Rpm_Loses15To25PercentOfTheFuelEnergy()
    {
        var cylinder = FiredCylinder();
        cylinder.RunCycle(Engine.CrankStepDeg, 3000);

        double fractionLost = cylinder.HeatLostJ / cylinder.HeatReleasedJ;

        Assert.InRange(fractionLost, 0.15, 0.25);
    }

    [Fact]
    public void LowRpm_LosesABiggerShareThanHighRpm()
    {
        // More time per cycle at low RPM, so the gas spends longer touching the walls
        var slow = FiredCylinder();
        var fast = FiredCylinder();
        slow.RunCycle(Engine.CrankStepDeg, 1000);
        fast.RunCycle(Engine.CrankStepDeg, 6000);

        Assert.True(slow.HeatLostJ / slow.HeatReleasedJ > fast.HeatLostJ / fast.HeatReleasedJ);
    }

    [Fact]
    public void GasAtWallTemperature_LosesNoHeat()
    {
        var heatTransfer = NewHeatTransfer();
        heatTransfer.CaptureReferenceState(StartPressurePa, VolumeAtBdcM3(), StartTemperatureK);

        double lossJ = heatTransfer.HeatLossJ(600, 500_000, Engine.WallTempK, 3000, Engine.CrankStepDeg, afterSpark: false);

        Assert.Equal(0, lossJ, precision: 12);
    }

    [Fact]
    public void PressureBelowTheNoBurnPressure_StillGivesARealHeatLoss()
    {
        // After the spark, heat loss can leave the gas slightly below the pressure it would have with no burn.
        // The combustion part of the gas speed must stop at zero rather than go negative (which gives NaN).
        var heatTransfer = NewHeatTransfer();
        heatTransfer.CaptureReferenceState(StartPressurePa, VolumeAtBdcM3(), StartTemperatureK);

        // At TDC the no-burn pressure is about 2,650 kPa: ask for the loss at only 1,000 kPa, at low RPM
        double lossJ = heatTransfer.HeatLossJ(720, 1_000_000, 900, 1000, Engine.CrankStepDeg, afterSpark: true);

        Assert.False(double.IsNaN(lossJ));
        Assert.True(lossJ > 0);
    }

    [Fact]
    public void MotoringWithHeatLoss_GivesSlightlyNegativeWork()
    {
        // No combustion: the gas is squeezed and let out again, but some of the squeezing heat leaks into the walls,
        // so the expansion gives back a little less than the compression took
        var cylinder = new Cylinder(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK, heatTransfer: NewHeatTransfer());

        double workJ = cylinder.RunCycle(Engine.CrankStepDeg, 3000);

        Assert.True(workJ < 0, $"work was {workJ} J");
        Assert.True(workJ > -0.1 * 273.9, $"work was {workJ} J"); // "slightly": under 10% of the ~274 J compression work
    }

    [Fact]
    public void EnergyCheck_StillHoldsWithHeatLoss()
    {
        var cylinder = FiredCylinder();
        double startEnergyJ = cylinder.InternalEnergyJ;

        double workJ = cylinder.RunCycle(Engine.CrankStepDeg, 3000);
        double energyChangeJ = cylinder.InternalEnergyJ - startEnergyJ;

        // First law: heat in − heat out either did work on the piston or is still in the gas
        Assert.Equal(cylinder.HeatReleasedJ - cylinder.HeatLostJ, workJ + energyChangeJ, precision: 6);
    }

    [Fact]
    public void WithoutHeatTransfer_NothingIsLost()
    {
        var cylinder = new Cylinder(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK,
            Combustion.FromSpark(20, StoichFuelG(), Fuel));

        cylinder.RunCycle(Engine.CrankStepDeg, 3000);

        Assert.Equal(0, cylinder.HeatLostJ);
    }

    // ---- Helpers ----

    private static HeatTransfer NewHeatTransfer() => new(Geometry, GasProperties.Air, Engine.WallTempK);

    // Full load: stoichiometric fuel, 20° spark advance, walls at 450 K
    private static Cylinder FiredCylinder() =>
        new(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK,
            Combustion.FromSpark(20, StoichFuelG(), Fuel), NewHeatTransfer());

    // Fuel for a stoichiometric mix with the air the cylinder traps (g)
    private static double StoichFuelG()
    {
        var unfired = new Cylinder(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK);
        return unfired.GasMassG / Fuel.StoichiometricAfr;
    }

    private static double VolumeAtBdcM3() =>
        new Cylinder(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK).VolumeM3;
}
