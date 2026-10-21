using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

public class Cylinder
{
    private readonly CylinderGeometry _cylinderGeometry;
    private readonly GasProperties _gasProperties;
    private readonly Combustion? _combustion;
    private readonly HeatTransfer? _heatTransfer;

    public double AirMassG { get; }
    private double AirMassKg => AirMassG / 1000; // R and cv are per kg, so the physics uses kg
    public double TempK { get; private set; }
    public double CrankAngleDeg { get; private set; }
    public double WorkJ { get; private set; }
    public double HeatReleasedJ { get; private set; }
    public double HeatLostJ { get; private set; }   // heat lost to the walls so far
    public double Rpm { get; set; }                 // crank speed, set from outside: heat loss depends on how long each step takes
    public double InternalEnergyJ => AirMassKg * _gasProperties.CvJPerKgK * TempK;
    public double VolumeM3 => VolumeHelper.CcToCubicMeters(_cylinderGeometry.Volume(CrankAngleDeg));
    public double PressurePa => AirMassKg * _gasProperties.GasConstantJPerKgK * TempK / VolumeM3;

    // Fill the cylinder at crankAngleDeg with gas at the given pressure and temperature, then seal it
    public Cylinder(CylinderGeometry geometry, GasProperties gas, double crankAngleDeg, double pressurePa, double temperatureK, Combustion? combustion = null, HeatTransfer? heatTransfer = null)
    {
        _cylinderGeometry = geometry;
        _gasProperties = gas;
        CrankAngleDeg = crankAngleDeg;
        TempK = temperatureK;
        // Ideal gas law again, solved for mass: m = p·V / (R·T) gives kg, × 1000 for grams
        AirMassG = pressurePa * VolumeM3 / (gas.GasConstantJPerKgK * temperatureK) * 1000;
        _combustion = combustion;
        _heatTransfer = heatTransfer;

        // Woschni needs the gas's state at the moment the valves shut: this one
        _heatTransfer?.CaptureReferenceState(pressurePa, VolumeM3, temperatureK);
    }

    // Turn the crank by stepDeg degrees, updating temperature and work (RK4)
    public void Step(double stepDeg)
    {
        double h = DegreeHelper.ToRadians(stepDeg);
        double theta = CrankAngleDeg;
        double t = TempK;
        double startPressure = PressurePa;

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

        // Heat leaking into the walls during this step cools the gas: ΔT = heat / (m·cv)
        if (_heatTransfer != null)
        {
            if (Rpm <= 0)
                throw new InvalidOperationException("Set Rpm before stepping a cylinder with heat transfer: heat loss depends on how long each step takes.");

            bool afterSpark = _combustion != null && theta >= _combustion.BurnStartDeg;
            double heatLossJ = _heatTransfer.HeatLossJ(theta, startPressure, t, Rpm, stepDeg, afterSpark);
            TempK -= heatLossJ / (AirMassKg * _gasProperties.CvJPerKgK);
            HeatLostJ += heatLossJ;
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
        double v = VolumeHelper.CcToCubicMeters(_cylinderGeometry.Volume(thetaDeg));      // m³
        double dV = VolumeHelper.CcToCubicMeters(_cylinderGeometry.DVdTheta(thetaDeg));  // m³ per radian
        double p = AirMassKg * _gasProperties.GasConstantJPerKgK * t / v;

        // Energy balance with no heat: m·cv·dT = −p·dV
        double dTdTheta = -p * dV / (AirMassKg * _gasProperties.CvJPerKgK);
        double dWdTheta = p * dV; // work by the gas: p·dV

        return (dTdTheta, dWdTheta);
    }
}
