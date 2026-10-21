using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

public class CylinderGeometry
{
    public double BoreMm { get; }               // Diameter of the cylinder
    public double StrokeLengthMm { get; }       // distance the piston moves up/down
    public double RodLengthMm { get; }          // l
    public double CompressionRatio { get; }
    public double CrankRadiusMm { get; }        // a = stroke / 2
    public double PistonAreaMm2 { get; }        // Math.PI / 4 × bore²
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

    // Cylinder volume (cc) at a crank angle in degrees. Steps 3–5.
    public double Volume(double crankAngleDeg)
    {
        double crankAngleRad = DegreeHelper.ToRadians(crankAngleDeg);
        double s = CrankRadiusMm * Math.Cos(crankAngleRad) + RodVerticalReach(crankAngleRad);
        double drop = RodLengthMm + CrankRadiusMm - s;
        return ClearanceVolumeCc + PistonAreaMm2 * drop / 1000;
    }

    // Piston travel per radian of crank (mm/rad). Step 7.
    public double LeverArm(double crankAngleDeg)
    {
        double crankAngleRad = DegreeHelper.ToRadians(crankAngleDeg);
        double a = CrankRadiusMm;
        return a * Math.Sin(crankAngleRad) * (1 + a * Math.Cos(crankAngleRad) / RodVerticalReach(crankAngleRad));
    }

    // Rate of volume change (cc per radian). Step 8.
    public double VolumeChangeCcPerRad(double crankAngleDeg)
    {
        return PistonAreaMm2 * LeverArm(crankAngleDeg) / 1000;
    }

    // Rod's vertical reach: Math.Sqrt(rodLength² − sideways²), step 2
    private double RodVerticalReach(double crankAngleRad)
    {
        double sideways = CrankRadiusMm * Math.Sin(crankAngleRad);
        return Math.Sqrt(RodLengthMm * RodLengthMm - sideways * sideways);
    }
}
