using EngineSim.Constants;
using EngineSim.Helpers;
using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.VehicleSystems.Air;

// Throttle body and intake manifold (plenum), #35.
// The throttle is a hole of fixed size between the outside air and the plenum. Air flows through it according to the
// pressure difference, and the engine pulls air out of the plenum. When the engine pulls faster than the hole can feed it,
// plenum pressure (MAP) drops, each cycle gets less air, and MAF levels off.
// The plenum air is assumed to stay at outside temperature ("isothermal"), so only its pressure changes.
public class IntakeManifold
{
    // Placeholders from ticket 35, tuned in the dyno ticket (#30)
    private const double ThrottleBoreMm = 70;
    private const double ClosedPlateAngleDeg = 7;          // plates sit slightly tilted when shut so they don't jam in the bore
    private const double ThrottleDischargeCoefficient = 0.8;
    // Air that always gets past the shut plate: real plates never seal perfectly and rest on a stop slightly open.
    // Kept small so the ECU has to hold the plate a little open to idle, and can open it more or less as needed (#32).
    public const double ClosedPlateLeakAreaMm2 = 10;
    private const double PlenumVolumeLiters = 4.0;         // about one displacement
    public const double DefaultSubStepSeconds = 0.001;     // the plenum reacts in ~20–50 ms, so step it at 1 ms, not once per 50 ms tick

    private readonly AmbientConditions _ambientConditions;
    private readonly double _displacementM3;
    private readonly double _subStepSeconds;

    public double PressurePa { get; private set; }             // plenum pressure right now
    public double MapKpa => PressurePa / 1000;                 // what a MAP sensor reads
    public double ThrottleFlowKgPerS { get; private set; }     // air coming in through the throttle: what the MAF reads
    public double MafGPerS => ThrottleFlowKgPerS * 1000;       // grams per second, as an OBD2 reader shows it
    public double EngineFlowKgPerS { get; private set; }       // air the engine is pulling out of the plenum
    private double PlenumVolumeM3 => PlenumVolumeLiters / 1000;

    // Plenum air density right now (kg/m³): pressure / (gas constant × temperature), at outside temperature
    public double DensityKgPerM3 => PressurePa / (GasProperties.Air.GasConstantJPerKgK * _ambientConditions.AirTempK);

    // subStepSeconds is only changed by tests, to check the result doesn't depend on the step size
    public IntakeManifold(AmbientConditions ambientConditions, double displacementLiters, double subStepSeconds = DefaultSubStepSeconds)
    {
        _subStepSeconds = subStepSeconds;
        _ambientConditions = ambientConditions;
        _displacementM3 = displacementLiters / 1000;
        PressurePa = ambientConditions.AirPressureKpa * 1000; // starts full: engine off, plenum at outside pressure
    }

    // Advance the plenum by dt seconds (one 50 ms Vehicle tick) in 1 ms substeps.
    // pedal: 0 = released, 1 = floored. rpm: crank speed, which sets how fast the engine pulls air out.
    public void Update(double dt, double rpm, double pedal)
    {
        int subSteps = (int)Math.Round(dt / _subStepSeconds);

        for (int i = 0; i < subSteps; i++)
        {
            // Mean-value mode: the engine's pull is worked out from rpm and the plenum pressure being tried
            SubStep(_subStepSeconds, pedal, plenumPa => EngineFlowAtKgPerS(plenumPa, rpm));
        }
    }

    // Crank-angle mode (#28): advance the plenum by one crank step, emptied by a real cylinder instead of the
    // mean-value guess. Call it right after cylinder.Step(stepDeg), once per crank step.
    //   seconds:              how long that crank step took: CrankHelper.StepSeconds(stepDeg, rpm)
    //   pedal:                plate opening, 0 = shut, 1 = wide open (pedal + ECU idle opening)
    //   cylinderInflowKgPerS: cylinder.IntakeFlowKgPerS. Positive = air leaving the plenum into the cylinder,
    //                         negative = the cylinder blowing gas back into the plenum
    // A crank step is only 0.03–0.2 ms, so this is one SubStep, not a 1 ms loop like Update.
    public void Step(double seconds, double pedal, double cylinderInflowKgPerS)
    {
        // The cylinder's flow stays the same for the whole step, whatever plenum pressure the bisection tries
        // (_ => means "take the pressure, don't use it"). The bisection still finds one answer: throttle flow in
        // drops as plenum pressure rises. SubStep sets PressurePa, ThrottleFlowKgPerS and EngineFlowKgPerS.
        SubStep(seconds, pedal, _ => cylinderInflowKgPerS);
    }

    // One small step, solved "implicitly": find the new plenum pressure where
    //     newPressurePa = PressurePa + seconds × PressureChangePaPerS(newPressurePa)
    // i.e. the pressure change is worked out from the flows at the END of the step, not the start.
    // With the throttle wide open the plenum fills in well under 1 ms, so working from the start of the step
    // overshoots past outside pressure and flips back and forth every substep. Solving for the end state can't overshoot.
    //
    // How: the left side minus the right side only ever rises as newPressurePa rises (more pressure = less air in through
    // the throttle and more air pulled out by the engine), so there's exactly one answer. Bisection finds it:
    // start with a range that must contain it, test the middle, keep the half the answer is in, repeat.
    //
    // engineFlowAtKgPerS: takes a plenum pressure (Pa), returns the kg/s leaving the plenum into the engine at that pressure.
    // Passed as a function so the same solver works for the mean-value guess (Update) and a real cylinder (Step).
    private void SubStep(double seconds, double pedal, Func<double, double> engineFlowAtKgPerS)
    {
        double outsidePressurePa = _ambientConditions.AirPressureKpa * 1000;
        double throttleAreaM2 = ThrottleAreaM2(pedal);

        // The answer is somewhere between (almost) vacuum and twice the higher of the old pressure and outside pressure.
        // It can end up above both: a cylinder blowing gas back through its intake valve pushes the plenum over outside pressure.
        double lowPa = 1;
        double highPa = 2 * Math.Max(PressurePa, outsidePressurePa);

        for (int i = 0; i < 60; i++)
        {
            double middlePa = (lowPa + highPa) / 2;
            double mismatchPa = middlePa - PressurePa - seconds * PressureChangePaPerS(middlePa, throttleAreaM2, engineFlowAtKgPerS);

            if (mismatchPa > 0)
                highPa = middlePa;  // guessed too high
            else
                lowPa = middlePa;   // guessed too low
        }

        PressurePa = (lowPa + highPa) / 2;
        ThrottleFlowKgPerS = ThrottleFlowAtKgPerS(PressurePa, throttleAreaM2);
        EngineFlowKgPerS = engineFlowAtKgPerS(PressurePa);
    }

    // How fast the plenum pressure would be changing (Pa/s) if it were at plenumPressurePa:
    //   gasConstant × outsideTempK / plenumVolumeM3 × (throttle flow in − engine flow out)
    // In words: more air in than out raises the pressure, and the smaller the plenum the faster it rises
    private double PressureChangePaPerS(double plenumPressurePa, double throttleAreaM2, Func<double, double> engineFlowAtKgPerS)
    {
        double netInflowKgPerS = ThrottleFlowAtKgPerS(plenumPressurePa, throttleAreaM2) - engineFlowAtKgPerS(plenumPressurePa);
        return GasProperties.Air.GasConstantJPerKgK * _ambientConditions.AirTempK / PlenumVolumeM3 * netInflowKgPerS;
    }

    // Outside air (side A) into the plenum (side B) through the throttle. The plenum air is at outside temperature too.
    private double ThrottleFlowAtKgPerS(double plenumPressurePa, double throttleAreaM2)
    {
        double outsideTempK = _ambientConditions.AirTempK;
        return GasHelper.MassFlowKgPerS(throttleAreaM2, _ambientConditions.AirPressureKpa * 1000, outsideTempK,
            plenumPressurePa, outsideTempK, GasProperties.Air, ThrottleDischargeCoefficient);
    }

    // Open area air can get through (m²): the gap around the throttle plate plus the closed-plate leak.
    //   plateAngleDeg  = PlateAngleDeg(pedal)
    //   plateGapMm2    = Math.PI / 4 × ThrottleBoreMm² × (1 − Math.Cos(plateAngle) / Math.Cos(closedPlateAngle))   (angles in radians)
    //   throttleAreaM2 = (plateGapMm2 + ClosedPlateLeakAreaMm2) / 1,000,000
    // Opening 0 gives the leak alone (the cosines cancel); opening 1 gives the full bore plus the leak.
    public double ThrottleAreaM2(double pedal)
    {
        double plateAngleDeg = PlateAngleDeg(pedal);
        // Math.Cos takes radians
        double plateGapMm2 = Math.PI / 4 * Math.Pow(ThrottleBoreMm, 2)
            * (1 - Math.Cos(DegreeHelper.ToRadians(plateAngleDeg)) / Math.Cos(DegreeHelper.ToRadians(ClosedPlateAngleDeg)));
        return (plateGapMm2 + ClosedPlateLeakAreaMm2) / 1000000;
    }

    // Plate angle for a pedal position: ClosedPlateAngleDeg at pedal 0, 90° (edge-on to the air, wide open) at pedal 1
    //   plateAngleDeg = ClosedPlateAngleDeg + pedal × (90 − ClosedPlateAngleDeg)
    private static double PlateAngleDeg(double pedal)
    {
        return ClosedPlateAngleDeg + pedal * (90 - ClosedPlateAngleDeg);
    }

    // Air the engine pulls out of the plenum (kg/s) if the plenum were at plenumPressurePa. Same as #5's formula, but with
    // plenum density instead of outside density, and no throttle factor: the throttle is the hole above now.
    //   plenumDensity    = plenumPressurePa / (gasConstant × outsideTempK)
    //   airPerCycleKg    = Engine.VolumetricEfficiency × plenumDensity × displacementM3
    //   engineFlowKgPerS = airPerCycleKg × rpm / 120          (a four-stroke fills every cylinder once per two revolutions)
    private double EngineFlowAtKgPerS(double plenumPressurePa, double rpm)
    {
        double plenumDensityKgPerM3 = plenumPressurePa / (GasProperties.Air.GasConstantJPerKgK * _ambientConditions.AirTempK);
        double airPerCycleKg = Engine.VolumetricEfficiency * plenumDensityKgPerM3 * _displacementM3;
        return airPerCycleKg * rpm / 120;
    }
}
