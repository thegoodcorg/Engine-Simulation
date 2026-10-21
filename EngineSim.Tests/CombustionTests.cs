using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #25: Wiebe heat release and spark timing in the sealed cylinder
public class CombustionTests
{
    // Cycle in the epic's convention: 0° / 720° = firing TDC, so compression starts at BDC = 540°
    private const double BdcDeg = 540;
    private const double FiringTdcDeg = 720;
    private const double StartPressurePa = 101_325;
    private const double StartTemperatureK = 288.15;

    private static readonly CylinderGeometry Geometry = new(95.5, 88.4, 144.2, 10.3);
    private static readonly FuelProperties Fuel = FuelProperties.PumpGas;

    // Ideal Otto-cycle efficiency, 1 − 1 / Math.Pow(compressionRatio, heatRatio − 1): the best a cycle with this compression ratio can do
    private static readonly double OttoEfficiency = 1 - 1 / Math.Pow(10.3, GasProperties.Air.HeatRatio - 1); // ~60.7%

    // ---- Wiebe burn fraction ----

    [Fact]
    public void BurnFraction_IsZeroBeforeAndAtTheSpark()
    {
        var combustion = new Combustion(700, 50, 1000);

        Assert.Equal(0, combustion.BurnFraction(600));
        Assert.Equal(0, combustion.BurnFraction(700));
    }

    [Fact]
    public void BurnFraction_AtTheEndOfTheBurn_Is1MinusEToTheMinus5()
    {
        var combustion = new Combustion(700, 50, 1000);

        Assert.Equal(1 - Math.Exp(-5), combustion.BurnFraction(750), precision: 6); // ~0.993
    }

    [Fact]
    public void BurnFraction_NeverDecreases()
    {
        var combustion = new Combustion(700, 50, 1000);

        double previous = 0;
        for (double angleDeg = 690; angleDeg <= 780; angleDeg += 0.5)
        {
            double fraction = combustion.BurnFraction(angleDeg);
            Assert.True(fraction >= previous, $"burn fraction went down at {angleDeg}°");
            previous = fraction;
        }
    }

    [Fact]
    public void HeatReleased_OverTheWholeBurn_IsAlmostAllTheFuelsHeat()
    {
        var combustion = new Combustion(700, 50, 1000);

        Assert.Equal(1000 * (1 - Math.Exp(-5)), combustion.HeatReleasedJ(600, 750), precision: 6);
    }

    [Fact]
    public void FromSpark_ConvertsAdvanceToBurnStartAndFuelToHeat()
    {
        var combustion = Combustion.FromSpark(20, 0.05, Fuel); // 20° advance, 50 mg of fuel

        Assert.Equal(700, combustion.BurnStartDeg);
        Assert.Equal(Engine.BurnDurationDeg, combustion.BurnDurationDeg);
        Assert.Equal(0.05 / 1000 * Fuel.EnergyContentJPerKg * Engine.CombustionEfficiency, combustion.TotalHeatJ, precision: 6); // 2,156 J
    }

    // ---- Combustion in the cylinder ----

    [Fact]
    public void NoCombustion_ReleasesNoHeat_AndGivesTheWorkBack()
    {
        // Spark off = no Combustion: the cylinder just compresses and expands like #24
        var cylinder = new Cylinder(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK);

        double workJ = cylinder.RunCycle(Engine.CrankStepDeg);

        Assert.Equal(0, cylinder.HeatReleasedJ);
        Assert.True(Math.Abs(workJ) < 0.01, $"net work was {workJ} J");
    }

    [Fact]
    public void InstantBurnAtTdc_MatchesOttoEfficiency()
    {
        // All the fuel burns in 1° right at TDC, the ideal Otto cycle
        var cylinder = CylinderWith(new Combustion(FiringTdcDeg, 1, StoichHeatJ()));

        double efficiency = cylinder.RunCycle(Engine.CrankStepDeg) / cylinder.HeatReleasedJ;

        Assert.Equal(1, efficiency / OttoEfficiency, 0.01); // within 1% of ~60.7%
    }

    [Fact]
    public void FiftyDegreeBurn_IsLessEfficientThanAnInstantBurn()
    {
        var instant = CylinderWith(new Combustion(FiringTdcDeg, 1, StoichHeatJ()));
        var normal = CylinderWith(Combustion.FromSpark(20, StoichFuelG(), Fuel));

        double instantEfficiency = instant.RunCycle(Engine.CrankStepDeg) / instant.HeatReleasedJ;
        double normalEfficiency = normal.RunCycle(Engine.CrankStepDeg) / normal.HeatReleasedJ;

        Assert.True(normalEfficiency < instantEfficiency); // ~58.9% vs ~60.7%
    }

    [Fact]
    public void SparkAtTdc_MakesLessWorkThan20DegreesAdvance()
    {
        // Late spark: most of the fuel burns after TDC, while the piston is already moving away
        Assert.True(WorkWithSpark(0) < WorkWithSpark(20)); // ~1,306 J vs ~1,485 J
    }

    [Fact]
    public void VeryEarlySpark_MakesLessWorkThan20DegreesAdvance()
    {
        // Early spark: pressure builds while the piston is still coming up, so it fights the compression
        Assert.True(WorkWithSpark(60) < WorkWithSpark(20)); // ~1,206 J vs ~1,485 J
    }

    [Fact]
    public void EnergyCheck_WorkPlusInternalEnergyChange_EqualsHeatReleased()
    {
        var cylinder = CylinderWith(Combustion.FromSpark(20, StoichFuelG(), Fuel));
        double startEnergyJ = cylinder.InternalEnergyJ;

        double workJ = cylinder.RunCycle(Engine.CrankStepDeg);
        double energyChangeJ = cylinder.InternalEnergyJ - startEnergyJ;

        // First law: the heat put in either did work on the piston or is still in the gas
        Assert.Equal(cylinder.HeatReleasedJ, workJ + energyChangeJ, precision: 6);
    }

    // ---- Helpers ----

    // Cylinder at BDC on a standard day, with the given burn
    private static Cylinder CylinderWith(Combustion combustion) =>
        new(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK, combustion);

    // Fuel for a stoichiometric mix with the air the cylinder traps (g).
    // The cylinder works out its own air mass, so build one without a burn just to read it.
    private static double StoichFuelG()
    {
        var unfired = new Cylinder(Geometry, GasProperties.Air, BdcDeg, StartPressurePa, StartTemperatureK);
        return unfired.GasMassG / Fuel.StoichiometricAfr;
    }

    // Heat that fuel releases (J), for building a Combustion directly
    private static double StoichHeatJ() =>
        StoichFuelG() / 1000 * Fuel.EnergyContentJPerKg * Engine.CombustionEfficiency;

    private static double WorkWithSpark(double sparkAdvanceDeg)
    {
        var cylinder = CylinderWith(Combustion.FromSpark(sparkAdvanceDeg, StoichFuelG(), Fuel));
        return cylinder.RunCycle(Engine.CrankStepDeg);
    }
}
