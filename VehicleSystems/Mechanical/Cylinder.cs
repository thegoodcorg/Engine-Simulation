using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

public class Cylinder
{
    private readonly CylinderGeometry _cylinderGeometry;
    private readonly GasProperties _gasProperties;
    private readonly Combustion? _combustion;

    public double AirMassG { get; }
    private double AirMassKg => AirMassG / 1000; // R and cv are per kg, so the physics uses kg
    public double TempK { get; private set; }
    public double CrankAngleDeg { get; private set; }
    public double WorkJ { get; private set; }
    public double HeatReleasedJ { get; private set; }
    public double InternalEnergyJ => AirMassKg * _gasProperties.CvJPerKgK * TempK;
    public double VolumeM3 => _cylinderGeometry.Volume(CrankAngleDeg) * 1e-6;
    public double PressurePa => AirMassKg * _gasProperties.GasConstantJPerKgK * TempK / VolumeM3;

    // Fill the cylinder at crankAngleDeg with gas at the given pressure and temperature, then seal it
    public Cylinder(CylinderGeometry geometry, GasProperties gas, double crankAngleDeg, double pressurePa, double temperatureK, Combustion? combustion = null)
    {
        _cylinderGeometry = geometry;
        _gasProperties = gas;
        CrankAngleDeg = crankAngleDeg;
        TempK = temperatureK;
        // Ideal gas law again, solved for mass: m = p·V / (R·T) gives kg, × 1000 for grams
        AirMassG = pressurePa * VolumeM3 / (gas.GasConstantJPerKgK * temperatureK) * 1000;
        _combustion = combustion;
    }

    // Turn the crank by stepDeg degrees, updating temperature and work (RK4)
    public void Step(double stepDeg)
    {
        double h = DegreeHelper.ToRadians(stepDeg);
        double theta = CrankAngleDeg;
        double t = TempK;

        // Four slope samples across the step: start, middle (twice), end
        var (dT1, dW1) = Slopes(theta, t);
        var (dT2, dW2) = Slopes(theta + stepDeg / 2, t + dT1 * h / 2);
        var (dT3, dW3) = Slopes(theta + stepDeg / 2, t + dT2 * h / 2);
        var (dT4, dW4) = Slopes(theta + stepDeg, t + dT3 * h);

        TempK = t + h / 6 * (dT1 + 2 * dT2 + 2 * dT3 + dT4);
        WorkJ += h / 6 * (dW1 + 2 * dW2 + 2 * dW3 + dW4);
        CrankAngleDeg = theta + stepDeg;

        if (_combustion != null)
        {
            double heatJ = _combustion.HeatReleasedJ(theta, CrankAngleDeg);
            TempK += heatJ / (AirMassKg * _gasProperties.CvJPerKgK);
            HeatReleasedJ += heatJ;
        }
    }

    // Run one sealed cycle from here (expects BDC, 540°) through firing TDC and back to BDC. Returns indicated work (J).
    public double RunCycle(double stepDeg)
    {
        int steps = (int)Math.Round(360 / stepDeg);
        for (int i = 0; i < steps; i++)
            Step(stepDeg);
        return WorkJ;
    }

    // How fast temperature and work change per radian of crank, at angle thetaDeg and temperature t
    private (double dTdTheta, double dWdTheta) Slopes(double thetaDeg, double t)
    {
        double v = _cylinderGeometry.Volume(thetaDeg) * 1e-6;     // m³
        double dV = _cylinderGeometry.DVdTheta(thetaDeg) * 1e-6;  // m³ per radian
        double p = AirMassKg * _gasProperties.GasConstantJPerKgK * t / v;

        // Energy balance with no heat: m·cv·dT = −p·dV
        double dTdTheta = -p * dV / (AirMassKg * _gasProperties.CvJPerKgK);
        double dWdTheta = p * dV; // work by the gas: p·dV

        return (dTdTheta, dWdTheta);
    }
}
