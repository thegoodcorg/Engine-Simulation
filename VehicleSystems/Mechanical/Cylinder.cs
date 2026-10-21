using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

public class Cylinder
{
    private readonly CylinderGeometry _cylinderGeometry;
    private readonly GasProperties _gasProperties;
    private readonly Combustion? _combustion;
    private readonly HeatTransfer? _heatTransfer;

    public double GasMassG { get; }
    private double AirMassKg => GasMassG / 1000; // R and cv are per kg, so the physics uses kg
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
        GasMassG = pressurePa * VolumeM3 / (gas.GasConstantJPerKgK * temperatureK) * 1000;
        _combustion = combustion;
        _heatTransfer = heatTransfer;

        // Woschni needs the gas's state at the moment the valves shut: this one
        _heatTransfer?.CaptureReferenceState(pressurePa, VolumeM3, temperatureK);
    }

    // Turn the crank by stepDeg degrees, updating temperature and work (RK4)
    public void Step(double stepDeg)
    {
        double h = DegreeHelper.ToRadians(stepDeg);
        double startAngleDeg = CrankAngleDeg;
        double t = TempK;
        double startPressure = PressurePa;

        // Four slope samples across the step: start, middle (twice), end
        var (dT1, dW1) = Slopes(startAngleDeg, t);
        var (dT2, dW2) = Slopes(startAngleDeg + stepDeg / 2, t + dT1 * h / 2);
        var (dT3, dW3) = Slopes(startAngleDeg + stepDeg / 2, t + dT2 * h / 2);
        var (dT4, dW4) = Slopes(startAngleDeg + stepDeg, t + dT3 * h);

        TempK = t + h / 6 * (dT1 + 2 * dT2 + 2 * dT3 + dT4);
        WorkJ += h / 6 * (dW1 + 2 * dW2 + 2 * dW3 + dW4);
        CrankAngleDeg = startAngleDeg + stepDeg;

        if (_combustion != null)
        {
            double heatJ = _combustion.HeatReleasedJ(startAngleDeg, CrankAngleDeg);
            TempK += heatJ / (AirMassKg * _gasProperties.CvJPerKgK);
            HeatReleasedJ += heatJ;
        }

        // Heat leaking into the walls during this step cools the gas: temperature drop = heat / (air mass × cv)
        if (_heatTransfer != null)
        {
            if (Rpm <= 0)
                throw new InvalidOperationException("Set Rpm before stepping a cylinder with heat transfer: heat loss depends on how long each step takes.");

            bool afterSpark = _combustion != null && startAngleDeg >= _combustion.BurnStartDeg;
            double heatLossJ = _heatTransfer.HeatLossJ(startAngleDeg, startPressure, t, Rpm, stepDeg, afterSpark);
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

    // How fast temperature and work change per radian of crank, at angle crankAngleDeg and temperature t
    private (double tempChangePerRad, double workPerRad) Slopes(double crankAngleDeg, double t)
    {
        double v = VolumeHelper.CcToCubicMeters(_cylinderGeometry.Volume(crankAngleDeg));      // m³
        double dV = VolumeHelper.CcToCubicMeters(_cylinderGeometry.VolumeChangeCcPerRad(crankAngleDeg));  // m³ per radian
        double p = AirMassKg * _gasProperties.GasConstantJPerKgK * t / v;

        // Energy balance with no heat: air mass × cv × temperature change = −pressure × volume change
        double tempChangePerRad = -p * dV / (AirMassKg * _gasProperties.CvJPerKgK);
        double workPerRad = p * dV; // work by the gas: pressure × volume change

        return (tempChangePerRad, workPerRad);
    }
}
