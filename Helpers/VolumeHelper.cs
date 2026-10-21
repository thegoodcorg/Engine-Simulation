namespace EngineSim.Helpers;

public class VolumeHelper
{
    private const double CubicMetersPerCc = 1e-6; // 1 m³ = 100 cm × 100 cm × 100 cm = 1,000,000 cc

    public static double CcToCubicMeters(double cc) => cc * CubicMetersPerCc;
}
