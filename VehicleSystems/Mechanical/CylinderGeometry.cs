using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

public class CylinderGeometry
{
    public double BoreMm { get; }               // Diameter of the cylinder
    public double StrokeLengthMm { get; }       // distance the piston moves up/down
    public double RodLengthMm { get; }          // l
    public double CompressionRatio { get; }
    public double CrankRadiusMm { get; }        // a = stroke / 2
    public double PistonAreaMm2 { get; }        // π/4 × bore²
    public double SweptVolumeCc { get; }        // Ap × stroke / 1000
    public double ClearanceVolumeCc { get; }    // swept / (CR − 1)

    public CylinderGeometry(double boreMm, double strokeMm, double rodLengthMm, double compressionRatio)
    {
        BoreMm = boreMm;
        StrokeLengthMm = strokeMm;
        RodLengthMm = rodLengthMm;
        CompressionRatio = compressionRatio;
        CrankRadiusMm = strokeMm / 2;
        PistonAreaMm2 = Math.PI / 4 * Math.Pow(boreMm, 2);
        SweptVolumeCc = PistonAreaMm2 * strokeMm / 1000;
        ClearanceVolumeCc = SweptVolumeCc / (compressionRatio - 1);
    }

    // Cylinder volume (cc) at crank angle θ in degrees. Steps 3–5.
    public double Volume(double thetaDeg)
    {
        double theta = DegreeHelper.ToRadians(thetaDeg);
        double s = CrankRadiusMm * Math.Cos(theta) + RodVerticalReach(theta);
        double drop = RodLengthMm + CrankRadiusMm - s;
        return ClearanceVolumeCc + PistonAreaMm2 * drop / 1000;
    }

    // Piston travel per radian of crank (mm/rad). Step 7.
    public double LeverArm(double thetaDeg)
    {
        double theta = DegreeHelper.ToRadians(thetaDeg);
        double a = CrankRadiusMm;
        return a * Math.Sin(theta) * (1 + a * Math.Cos(theta) / RodVerticalReach(theta));
    }

    // Rate of volume change (cc per radian). Step 8.
    public double DVdTheta(double thetaDeg)
    {
        return PistonAreaMm2 * LeverArm(thetaDeg) / 1000;
    }

    // √(l² − a²·sin²θ), step 2
    private double RodVerticalReach(double theta)
    {
        double sideways = CrankRadiusMm * Math.Sin(theta);
        return Math.Sqrt(RodLengthMm * RodLengthMm - sideways * sideways);
    }
}
