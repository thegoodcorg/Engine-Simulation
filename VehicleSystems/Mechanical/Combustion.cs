namespace EngineSim.VehicleSystems.Mechanical;

public class Combustion
{
    public double BurnStartDeg { get; }
    public double BurnDurationDeg { get; }
    public double TotalHeatJ { get; }  // total energy released during fuel burn

    public Combustion(double burnStartDeg, double burnDurationDeg, double totalHeatJ)
    {
        BurnStartDeg = burnStartDeg;
        BurnDurationDeg = burnDurationDeg;
        TotalHeatJ = totalHeatJ;
    }

    public static Combustion FromSpark(double SparkAdvanceDeg, double fuelG, FuelProperties fuel)
    {
        double fuelKg = fuelG / 1000;
        // the amount of fuel(Kg) * its energy density * the engine combustion efficiency
        double totalHeatJ = fuelKg * fuel.EnergyContentJPerKg * Engine.CombustionEfficiency;

        return new Combustion(720 - SparkAdvanceDeg, Engine.BurnDurationDeg, totalHeatJ);
    }

    // Fraction of fuel burned by crankAngleDeg, for a burn that starts at BurnStartDeg and lasts BurnDurationDeg: 
    public double BurnFraction(double crankAngleDeg)
    {
        if (crankAngleDeg <= BurnStartDeg) return 0;

        double progress = (crankAngleDeg - BurnStartDeg) / BurnDurationDeg; // 0 at spark up to 1 at the end of the burn


        return 1 - Math.Exp(-Engine.WiebeA * Math.Pow(progress, Engine.WiebeM + 1));     // burnFraction = 1 − Math.Exp(−WiebeA × Math.Pow(progress, WiebeM + 1))

    }

    public double HeatReleasedJ(double fromDeg, double toDeg)
    {
        return TotalHeatJ * (BurnFraction(toDeg) - BurnFraction(fromDeg));
    }
}