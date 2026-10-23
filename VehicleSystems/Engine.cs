using EngineSim.Constants;
using EngineSim.Enums;
using EngineSim.Helpers;
using EngineSim.VehicleSystems.Air;
using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim;

public class Engine
{
    // Tuning constants
    private const double PistonSpeedLimitMs = 20;        // max mean piston speed, sets the redline
    private const double InertiaPerLiter = 0.05;            // kg·m² per liter of displacement
    public const double CrankTimeSeconds = 0.5;
    public const double CrankStepDeg = 0.5;             // crank angle per simulation step for the cylinder model
    public const double WiebeA = 5;                      // burn completeness: 1 − e^(−5) = 99.3% of the fuel burned by the end of the burn duration
    public const double WiebeM = 2;                      // burn shape: slow start while the flame kernel grows, fast middle, long tail as it reaches the walls
    public const double BurnDurationDeg = 50;
    public const double CombustionEfficiency = 0.98;
    public const double ExhaustBackPressureAboveAmbientPa = 5_000; // exhaust system back-pressure: exhaust port = ambient + 5 kPa
    public const double WallTempK = 450;                    // fixed, will replace later once cooling code comes into play.
    private const double FrictionBaseBar = 0.97;           // always there: seals, accessories
    private const double FrictionPerKrpmBar = 0.15;        // grows steadily with speed: bearings, rings
    private const double FrictionPerKrpmSquaredBar = 0.05; // grows fast at high RPM: oil churning, valvetrain
    public const double PeakTorqueRpm = 4400;              // where the VQ38DD makes peak torque (Nissan spec, #30's calibration target)
    public EngineLayout Layout { get; }   // inline, V or flat: a label for now, it doesn't change the firing angles (#29)

    public int CylinderCount { get; }
    public double FiringIntervalDeg => 720.0 / CylinderCount;   // crank degrees between one cylinder firing and the next: every engine is even-firing

    // The crank-angle model's parts (#29). One crankshaft turns every cylinder; they all breathe from one plenum
    // and share one set of cam profiles, each cylinder reading the cams at its own crank angle.
    public IReadOnlyList<Cylinder> Cylinders { get; }
    public Valvetrain Valvetrain { get; }
    public IntakeManifold IntakeManifold { get; }

    // Crank degrees per substep. CrankStepDeg unless a test sets a finer one to check the answer doesn't depend on it.
    public double StepDeg { get; init; } = CrankStepDeg;

    public double CrankAngleDeg { get; private set; }   // the master crank angle every cylinder follows, keeps counting up (720 = one cycle)
    public double InstantTorqueNm { get; private set; } // net torque on the crank in the last substep: ripples with every firing
    public double RealTimeFactor { get; private set; }  // simulated seconds ÷ real seconds for the last Update: under 1 = slower than real time

    // Spark advance in degrees before TDC, copied into every cylinder. null = spark off: no fuel injected and nothing burns.
    // Set from the ECU by Vehicle each tick.
    public double? SparkAdvanceDeg { get; set; }
    public double CylinderVolumeLiters { get; }
    public double DisplacementLiters { get; }
    public double MaxRpm { get; }
    public double EngineSpeed { get; private set; }
    public double EngineTorqueNm { get; private set; } // torque from the gas on the pistons, averaged over the last Update (before friction)
    public double NetTorque { get; private set; }      // EngineTorqueNm − friction, averaged over the last Update: what reaches the flywheel
    public double Inertia { get; }
    public bool IsRunning { get; private set; }
    public double FuelBurnedG { get; private set; } // fuel injected in the last Update (g), for the Vehicle to drain from the tank
    private readonly AmbientConditions _ambientConditions;
    private readonly FuelProperties _fuel;
    public double MafGPerS => IntakeManifold.MafGPerS;                 // air coming in through the throttle: what a MAF sensor reads, passed to the ECU
    public double ManifoldPressureKpa => IntakeManifold.MapKpa;        // plenum pressure (MAP), absolute
    public double OutsideAirDensityKgM3 => _ambientConditions.AirDensityKgM3;   // the air the whole vehicle drives through (aero drag uses it)

    public CylinderGeometry Geometry { get; }  // bore, stroke, rod length, compression ratio and volume vs crank angle

    // 0 = released (idle air still gets in through the manifold's bypass), 1.0 = wide open. Values outside that range get clamped.
    private double _throttlePosition = 0;
    public double ThrottlePosition
    {
        get => _throttlePosition;
        set => _throttlePosition = Math.Clamp(value, 0, 1);
    }

    public Engine(
        double boreMm,
        double strokeMm,
        double conRodLengthMm,
        double compressionRatio,
        int cylinders,
        AmbientConditions ambientConditions,
        FuelProperties fuel,
        EngineLayout layout = EngineLayout.V)
    {
        // Built first: displacement and redline below read from it
        Geometry = new CylinderGeometry(boreMm, strokeMm, conRodLengthMm, compressionRatio);
        CylinderCount = cylinders;
        CylinderVolumeLiters = Geometry.SweptVolumeCc / 1000; // cc → liters
        DisplacementLiters = TotalDisplacement();
        Layout = layout;
        MaxRpm = CalculateRedline();
        EngineSpeed = 0; // starts switched off
        Inertia = InertiaPerLiter * DisplacementLiters;
        _ambientConditions = ambientConditions;
        _fuel = fuel;
        IntakeManifold = new IntakeManifold(ambientConditions, DisplacementLiters);
        Valvetrain = new Valvetrain();
        Cylinders = BuildCylinders();
    }

    // One Cylinder per cylinder, each started at its own place in the cycle: cylinder 1 at 0° (firing TDC),
    // and each one after it FiringIntervalDeg behind the one before (−120°, −240° … on a V6), wrapped into 0–720°.
    // They all start full of outside air. Each gets its own HeatTransfer, because it remembers that cylinder's state at IVC.
    private List<Cylinder> BuildCylinders()
    {
        double outsidePressurePa = _ambientConditions.AirPressureKpa * 1000;
        var cylinders = new List<Cylinder>();

        for (int i = 0; i < CylinderCount; i++)
        {
            double startAngleDeg = ((-i * FiringIntervalDeg) % 720 + 720) % 720;

            cylinders.Add(new Cylinder(Geometry, GasProperties.Air, startAngleDeg, outsidePressurePa, _ambientConditions.AirTempK,
                heatTransfer: new HeatTransfer(Geometry, GasProperties.Air, WallTempK),
                valvetrain: Valvetrain,
                fuel: _fuel)
            {
                ExhaustPressurePa = outsidePressurePa + ExhaustBackPressureAboveAmbientPa,
            });
        }

        return cylinders;
    }

    // The speed the starter turns the crank at, and the least an engine leaves the starter at.
    public const double CrankingRpm = 200;

    // Below this the crank has as good as stopped: it's set to 0 instead of being stepped.
    // A crank-angle step is a fixed half degree, so the slower the crank, the longer each step lasts. The valve flow is
    // worked out once per step, and below ~130 RPM a step is long enough for it to move more gas than the cylinder holds,
    // which blows up to NaN. 150 leaves a margin, and stays under CrankingRpm (and the ECU's 250 RPM stall check).
    private const double MinTurningRpm = 150;

    // The starter is turning the crank (crank-angle model, #29). While it is, the crank never drops below CrankingRpm,
    // so the cylinders pump, fill and fire before the starter lets go. It can't slow a crank that's already faster.
    // Vehicle sets it from the Starter every tick.
    public bool StarterEngaged { get; set; }

    public void StartEngine()
    {
        IsRunning = true;
        EngineSpeed = Math.Max(EngineSpeed, CrankingRpm);
    }

    // Engine stops because something went wrong (battery cut, fuel ran out)
    public void Stall()
    {
        IsRunning = false;
    }

    // Engine stops because the driver turned the key off
    public void Shutdown()
    {
        IsRunning = false;
    }

    // Throttle plate opening (0–1) this tick: the pedal plus whatever the ECU adds to hold idle.
    // Drive-by-wire: the pedal is only a request, and the ECU decides where the plate actually sits.
    public double ThrottlePlateOpening { get; private set; }

    // Hold the crank at a fixed RPM whatever torque the engine makes, like a dyno absorbing or supplying the difference
    // (null = free running). Torques are still worked out normally, so NetTorque shows what the engine would do.
    // Used by tests and the dyno (#30). Setting it puts the crank at that speed straight away.
    public double? HeldRpm
    {
        get => _heldRpm;
        set
        {
            _heldRpm = value;
            if (value is double rpm)
                EngineSpeed = rpm;
        }
    }
    private double? _heldRpm;

    // Advance the engine by dt seconds (one 0.05 s Vehicle tick), in crank-angle substeps of StepDeg (#29).
    // Torque comes from the gas pressure in each cylinder. Each cylinder injects fuel for the air it actually
    // trapped, at its own intake valve closing, so there's no fuel-per-cycle input.
    // idleThrottleOpening is what the ECU adds to the pedal. loadTorqueNm and extraInertiaKgM2 come from the drivetrain (#38):
    // what the vehicle feels like at the crank. Both 0 when nothing's connected (neutral or clutch in).
    public void Update(double dt, double idleThrottleOpening, double loadTorqueNm = 0, double extraInertiaKgM2 = 0)
    {
        ThrottlePlateOpening = Math.Clamp(ThrottlePosition + idleThrottleOpening, 0, 1);

        // The PCV system vents the crankcase, so the underside of each piston sits at outside pressure. kPa × 1000 = Pa
        double crankCasePressurePa = _ambientConditions.AirPressureKpa * 1000;
        double outsideAirTempK = _ambientConditions.AirTempK;   // the plenum stays at outside temperature

        double fuelBeforeG = Cylinders.Sum(c => c.FuelInjectedG);   // to tell how much was injected this tick
        var stopwatch = System.Diagnostics.Stopwatch.StartNew();    // for RealTimeFactor

        // Running totals for the tick's averages: torque × seconds, added up, ÷ total seconds = time-averaged torque
        double elapsedSeconds = 0;
        double torqueTimesSeconds = 0;
        double netTorqueTimesSeconds = 0;

        // The starter gets a stopped crank turning
        if (StarterEngaged)
            EngineSpeed = Math.Max(EngineSpeed, CrankingRpm);

        // A stopped crank has nothing to step (StepSeconds divides by RPM, so the step would take forever)
        while (elapsedSeconds < dt && EngineSpeed > 0)
        {
            double stepSeconds = CrankHelper.StepSeconds(StepDeg, EngineSpeed);
            double totalIntakeFlowKgPerS = 0;
            double cylinderTorqueNm = 0;

            // 1. Every cylinder breathes from the plenum and pushes on the crank
            foreach (Cylinder cylinder in Cylinders)
            {
                // Spark (and fuel) while running, and while cranking so it can catch
                cylinder.SparkAdvanceDeg = IsRunning || StarterEngaged ? SparkAdvanceDeg : null;
                cylinder.IntakePressurePa = IntakeManifold.PressurePa;
                cylinder.IntakeTempK = outsideAirTempK;
                cylinder.Step(StepDeg, EngineSpeed);
                totalIntakeFlowKgPerS += cylinder.IntakeFlowKgPerS;
                cylinderTorqueNm += cylinder.TorqueNm(crankCasePressurePa);
            }

            // 2. The plenum: filled by the throttle, emptied by the intake valves
            IntakeManifold.Step(stepSeconds, ThrottlePlateOpening, totalIntakeFlowKgPerS);

            // 3. Net torque. No separate pumping loss: the real cylinder pressures already include it.
            InstantTorqueNm = cylinderTorqueNm - FrictionTorqueNm(EngineSpeed);

            // 4. Rotational version of F = ma: angular acceleration = torque / inertia (rad/s²), then rad/s → RPM
            double angularAccelerationRadPerS2 = (InstantTorqueNm - loadTorqueNm) / (Inertia + extraInertiaKgM2);
            EngineSpeed += angularAccelerationRadPerS2 * 60 / (2 * Math.PI) * stepSeconds;

            // No idle floor: idle is held by the ECU's throttle plate opening, so RPM can fall to 0 (and the ECU stalls the engine).
            // MaxRpm stays as a hard ceiling, because a tick at full throttle can overshoot the rev limiter before it cuts spark.
            EngineSpeed = Math.Clamp(EngineSpeed, 0, MaxRpm);

            // Starter turning: it holds the crank up at cranking speed. Otherwise a crank this slow has stopped (the loop ends).
            if (StarterEngaged)
                EngineSpeed = Math.Max(EngineSpeed, CrankingRpm);
            else if (EngineSpeed < MinTurningRpm)
                EngineSpeed = 0;

            // Held (dyno or test): put the crank back to the held speed whatever the torque did
            if (_heldRpm is double heldRpm)
            {
                EngineSpeed = heldRpm;
            }

            // 5. The crank always moves on, held or not
            CrankAngleDeg += StepDeg;

            torqueTimesSeconds += cylinderTorqueNm * stepSeconds;
            netTorqueTimesSeconds += InstantTorqueNm * stepSeconds;
            elapsedSeconds += stepSeconds;
        }

        if (elapsedSeconds > 0)
        {
            EngineTorqueNm = torqueTimesSeconds / elapsedSeconds;
            NetTorque = netTorqueTimesSeconds / elapsedSeconds;
            FuelBurnedG = Cylinders.Sum(c => c.FuelInjectedG) - fuelBeforeG;
            RealTimeFactor = elapsedSeconds / stopwatch.Elapsed.TotalSeconds;
        }
        else
        {
            // Stopped crank: nothing made, nothing used
            EngineTorqueNm = 0;
            NetTorque = 0;
            InstantTorqueNm = 0;
            FuelBurnedG = 0;
        }
    }

    private double TotalDisplacement()
    {
        return CylinderVolumeLiters * CylinderCount;
    }

    private double CalculateRedline() // TODO: this will eventually die for a better simulation of the redline
    {
        double maxRpm = 30 * PistonSpeedLimitMs / Geometry.StrokeLengthMm * 1000; // multiply by 1000 to get from mm on stroke length to meters on the piston speed

        return Math.Floor(maxRpm / 100) * 100; // round down to the closest 100 IE 8769 -> 8700
    }

    // Torque from a mean effective pressure: pressure(Pa) * displacement(m^3) / (4pi)
    // Used for FMEP (friction torque)
    private double PressureToTorqueNm(double pressureBar)
    {
        double pressureInPa = pressureBar * 100000;

        double displacementInMCubed = DisplacementLiters / 1000;

        return pressureInPa * displacementInMCubed / (4 * Math.PI);
    }

    // Friction torque at a given RPM: FMEP (bar) turned into torque, the same conversion as before
    public double FrictionTorqueNm(double rpm) => PressureToTorqueNm(CalculateFmepBar(rpm));

    // Friction mean effective pressure (bar) at a given RPM (Heywood, spark-ignition engines):
    //   fmepBar = base + perKrpm × rpmK + perKrpmSquared × rpmK², where rpmK = RPM in thousands
    private static double CalculateFmepBar(double rpm)
    {
        double rpmK = rpm / 1000;
        return FrictionBaseBar + FrictionPerKrpmBar * rpmK + FrictionPerKrpmSquaredBar * rpmK * rpmK;
    }
}
