using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

// One cylinder's gas, stepped through crank angle.
// Sealed (no Valvetrain): valves always shut, as in #24–#26.
// Open (with a Valvetrain, #27): gas flows in and out through the valves, and it runs cycle after cycle,
// with the exhaust left behind at the end of one cycle carried into the next.
public class Cylinder
{
    private const double CycleDeg = 720; // one four-stroke cycle: two crank revolutions

    private readonly CylinderGeometry _cylinderGeometry;
    private readonly GasProperties _gasProperties;
    private Combustion? _combustion;                // sealed mode: given once. Open mode: rebuilt at every IVC
    private readonly HeatTransfer? _heatTransfer;
    private readonly Valvetrain? _valvetrain;
    private readonly FuelProperties? _fuel;

    public double GasMassG { get; private set; }    // all the gas in the cylinder: fresh air plus leftover exhaust
    private double GasMassKg => GasMassG / 1000;    // the gas constant and cv are per kg, so the physics uses kg
    public double TempK { get; private set; }
    public double CrankAngleDeg { get; private set; }
    public double WorkJ { get; private set; }
    public double HeatReleasedJ { get; private set; }
    public double HeatLostJ { get; private set; }   // heat lost to the walls so far
    public double Rpm { get; set; }                 // crank speed, set from outside: heat loss and valve flow depend on how long each step takes
    public double InternalEnergyJ => GasMassKg * _gasProperties.CvJPerKgK * TempK;
    public double VolumeM3 => VolumeHelper.CcToCubicMeters(_cylinderGeometry.Volume(CrankAngleDeg));
    public double PressurePa => GasMassKg * _gasProperties.GasConstantJPerKgK * TempK / VolumeM3;

    // ---- Open mode only ----

    // What's on the other side of each valve. Intake is ambient for now; the plenum sets it in #28.
    public double IntakePressurePa { get; set; } = 101_325;
    public double IntakeTempK { get; set; } = 288.15;
    public double ExhaustPressurePa { get; set; } = 101_325 + Engine.ExhaustBackPressureAboveAmbientPa;

    // Spark advance in degrees before firing TDC. null = spark off: no fuel and no burn.
    public double? SparkAdvanceDeg { get; set; }
    public double IntakeFlowKgPerS { get; private set; }   // air in through the intake valve this step: + = into the cylinder, − = blowing back into the plenum

    // Gas that backflows from the exhaust comes in at the average temperature of what last went out
    private double _exhaustPortTempK = 900;

    // Bookkeeping for the cycle in progress
    private double _residualMassG;      // recorded at EVC
    private double _trappedMassG;       // recorded at IVC
    private double _intakeNetG;         // net gas in through the intake valve
    private double _exhaustNetG;        // net gas out through the exhaust valve
    private double _exhaustOutG;        // gas that went out the exhaust, and its mass × temperature,
    private double _exhaustOutGTimesK;  // for the average exhaust temperature

    // Fill the cylinder at crankAngleDeg with gas at the given pressure and temperature.
    // Without a valvetrain it's sealed from here on. With one, it breathes: start it at 0° (firing TDC) and run whole cycles.
    public Cylinder(CylinderGeometry geometry, GasProperties gas, double crankAngleDeg, double pressurePa, double temperatureK,
        Combustion? combustion = null, HeatTransfer? heatTransfer = null, Valvetrain? valvetrain = null, FuelProperties? fuel = null)
    {
        _cylinderGeometry = geometry;
        _gasProperties = gas;
        CrankAngleDeg = crankAngleDeg;
        TempK = temperatureK;
        // Ideal gas law again, solved for mass: mass = pressure × volume / (gas constant × temperature) gives kg, × 1000 for grams
        GasMassG = pressurePa * VolumeM3 / (gas.GasConstantJPerKgK * temperatureK) * 1000;
        _combustion = combustion;
        _heatTransfer = heatTransfer;
        _valvetrain = valvetrain;
        _fuel = fuel;

        // Woschni needs the gas's state at the moment the valves shut: this one (open mode re-captures it at every IVC)
        _heatTransfer?.CaptureReferenceState(pressurePa, VolumeM3, temperatureK);
    }

    // Turn the crank by stepDeg degrees, updating temperature and work (RK4), then heat and valve flow
    public void Step(double stepDeg)
    {
        IntakeFlowKgPerS = 0;
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
            TempK += heatJ / (GasMassKg * _gasProperties.CvJPerKgK);
            HeatReleasedJ += heatJ;
        }

        // Heat leaking into the walls during this step cools the gas: temperature drop = heat / (gas mass × cv)
        if (_heatTransfer != null)
        {
            if (Rpm <= 0)
                throw new InvalidOperationException("Set Rpm before stepping a cylinder with heat transfer: heat loss depends on how long each step takes.");

            // Woschni's combustion turbulence only counts while burning and expanding with both valves shut
            bool afterSpark = _combustion != null && startAngleDeg >= _combustion.BurnStartDeg && ValvesShut(startAngleDeg);
            double heatLossJ = _heatTransfer.HeatLossJ(startAngleDeg, startPressure, t, Rpm, stepDeg, afterSpark);
            TempK -= heatLossJ / (GasMassKg * _gasProperties.CvJPerKgK);
            HeatLostJ += heatLossJ;
        }

        if (_valvetrain != null)
        {
            FlowThroughValves(startAngleDeg, stepDeg);
            CheckValveEvents(startAngleDeg, CrankAngleDeg);
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

    // Open mode: run one full 720° cycle from here and return its numbers
    public CycleResult RunOpenCycle(double stepDeg)
    {
        if (_valvetrain == null)
            throw new InvalidOperationException("RunOpenCycle needs a Valvetrain. Use RunCycle for a sealed cylinder.");

        double startWorkJ = WorkJ;
        _intakeNetG = 0;
        _exhaustNetG = 0;
        _exhaustOutG = 0;
        _exhaustOutGTimesK = 0;

        int steps = (int)Math.Round(CycleDeg / stepDeg);
        for (int i = 0; i < steps; i++)
            Step(stepDeg);

        // Next cycle's exhaust backflow comes in at the average temperature of what went out this cycle
        if (_exhaustOutG > 0)
            _exhaustPortTempK = _exhaustOutGTimesK / _exhaustOutG;

        // VE: fresh air trapped ÷ the air that would fill the swept volume at intake density (density = pressure / (gas constant × temperature))
        double intakeDensityKgPerM3 = IntakePressurePa / (_gasProperties.GasConstantJPerKgK * IntakeTempK);
        double perfectFillG = intakeDensityKgPerM3 * VolumeHelper.CcToCubicMeters(_cylinderGeometry.SweptVolumeCc) * 1000;
        double freshAirG = _trappedMassG - _residualMassG;

        return new CycleResult(
            IndicatedWorkJ: WorkJ - startWorkJ,
            TrappedMassG: _trappedMassG,
            ResidualMassG: _residualMassG,
            ResidualGasFraction: _residualMassG / _trappedMassG,
            VolumetricEfficiency: freshAirG / perfectFillG,
            MassInG: _intakeNetG,
            MassOutG: _exhaustNetG);
    }

    // Open mode: repeat cycles until indicated work and trapped mass each change by under 0.5% from one cycle to the next
    public CycleResult RunUntilSettled(double stepDeg, int maxCycles = 20)
    {
        CycleResult previous = RunOpenCycle(stepDeg);
        for (int i = 1; i < maxCycles; i++)
        {
            CycleResult current = RunOpenCycle(stepDeg);
            if (ChangedUnderHalfAPercent(previous.IndicatedWorkJ, current.IndicatedWorkJ)
                && ChangedUnderHalfAPercent(previous.TrappedMassG, current.TrappedMassG))
                return current;
            previous = current;
        }
        return previous;
    }

    private static bool ChangedUnderHalfAPercent(double before, double after) =>
        Math.Abs(after - before) <= 0.005 * Math.Max(Math.Abs(after), 1e-9);

    // Gas through both valves during this step. Both flows are worked out from the same cylinder state, then applied together.
    private void FlowThroughValves(double startAngleDeg, double stepDeg)
    {
        double stepSeconds = CrankHelper.StepSeconds(stepDeg, Rpm);
        double cylinderPressurePa = PressurePa;
        double cylinderTempK = TempK;

        // Side A = the port, side B = the cylinder, so positive = into the cylinder
        double intakeKg = GasHelper.MassFlowKgPerS(_valvetrain!.Intake.FlowAreaM2(startAngleDeg),
            IntakePressurePa, IntakeTempK, cylinderPressurePa, cylinderTempK, _gasProperties, Valvetrain.DischargeCoefficient) * stepSeconds;

        IntakeFlowKgPerS = intakeKg / stepSeconds;

        double exhaustKg = GasHelper.MassFlowKgPerS(_valvetrain.Exhaust.FlowAreaM2(startAngleDeg),
            ExhaustPressurePa, _exhaustPortTempK, cylinderPressurePa, cylinderTempK, _gasProperties, Valvetrain.DischargeCoefficient) * stepSeconds;

        // Energy the gas carries across a valve = kg × cp × the temperature of the side it came from
        double energyJ = InternalEnergyJ;
        energyJ += intakeKg * _gasProperties.CpJPerKgK * (intakeKg > 0 ? IntakeTempK : cylinderTempK);
        energyJ += exhaustKg * _gasProperties.CpJPerKgK * (exhaustKg > 0 ? _exhaustPortTempK : cylinderTempK);

        double newMassKg = GasMassKg + intakeKg + exhaustKg;
        GasMassG = newMassKg * 1000;
        TempK = energyJ / (newMassKg * _gasProperties.CvJPerKgK);

        _intakeNetG += intakeKg * 1000;
        _exhaustNetG -= exhaustKg * 1000;
        if (exhaustKg < 0)
        {
            _exhaustOutG += -exhaustKg * 1000;
            _exhaustOutGTimesK += -exhaustKg * 1000 * cylinderTempK;
        }
    }

    // EVC: whatever is still in the cylinder is this cycle's residual exhaust.
    // IVC: the charge is trapped, so record it, re-capture Woschni's reference state, and set up this cycle's burn.
    private void CheckValveEvents(double fromDeg, double toDeg)
    {
        if (Crossed(fromDeg, toDeg, _valvetrain!.Exhaust.ClosesDeg))
            _residualMassG = GasMassG;

        if (Crossed(fromDeg, toDeg, _valvetrain.Intake.ClosesDeg))
        {
            _trappedMassG = GasMassG;
            _heatTransfer?.CaptureReferenceState(PressurePa, VolumeM3, TempK);

            _combustion = null;
            if (SparkAdvanceDeg is double advanceDeg && _fuel != null)
            {
                double freshAirG = Math.Max(_trappedMassG - _residualMassG, 0);
                double fuelG = freshAirG / _fuel.StoichiometricAfr;

                // This cycle's firing TDC is the next multiple of 720° after IVC. FromSpark assumes 720°, so shift the burn to this cycle.
                double firingTdcDeg = Math.Ceiling(toDeg / CycleDeg) * CycleDeg;
                Combustion burn = Combustion.FromSpark(advanceDeg, fuelG, _fuel);
                _combustion = new Combustion(firingTdcDeg - advanceDeg, burn.BurnDurationDeg, burn.TotalHeatJ);
            }
        }
    }

    // Did the crank pass eventDeg (in any cycle) on the way from fromDeg to toDeg?
    private static bool Crossed(double fromDeg, double toDeg, double eventDeg)
    {
        double sinceEventDeg = ((fromDeg - eventDeg) % CycleDeg + CycleDeg) % CycleDeg; // how far past the event we were at the start
        return sinceEventDeg == 0 || sinceEventDeg + (toDeg - fromDeg) >= CycleDeg;
    }

    private bool ValvesShut(double crankAngleDeg) =>
        _valvetrain == null
        || (_valvetrain.Intake.FlowAreaM2(crankAngleDeg) == 0 && _valvetrain.Exhaust.FlowAreaM2(crankAngleDeg) == 0);

    // How fast temperature and work change per radian of crank, at angle crankAngleDeg and temperature t
    private (double tempChangePerRad, double workPerRad) Slopes(double crankAngleDeg, double t)
    {
        double v = VolumeHelper.CcToCubicMeters(_cylinderGeometry.Volume(crankAngleDeg));      // m³
        double dV = VolumeHelper.CcToCubicMeters(_cylinderGeometry.VolumeChangeCcPerRad(crankAngleDeg));  // m³ per radian
        double p = GasMassKg * _gasProperties.GasConstantJPerKgK * t / v;

        // Energy balance with no heat: gas mass × cv × temperature change = −pressure × volume change
        double tempChangePerRad = -p * dV / (GasMassKg * _gasProperties.CvJPerKgK);
        double workPerRad = p * dV; // work by the gas: pressure × volume change

        return (tempChangePerRad, workPerRad);
    }
}
