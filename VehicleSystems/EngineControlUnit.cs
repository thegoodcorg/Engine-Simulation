namespace EngineSim;

// The answer to "may the engine run?": allowed or not, plus a message to show.
// Message is the reason when not allowed, a warning when allowed (e.g. low fuel), or null when all is well.
public record RunCheckResult(bool Allowed, string? Message);

// Everything the ECU reads in one tick. The Vehicle fills this in from the sensors.
public record EcuInputs(
    double Rpm,
    double MaxRpm,
    bool BatteryConnected,
    double FuelLevelPercentage,
    bool EngineRunning,
    bool Cranking,
    double MafGPerS,                 // mass airflow (MAF) reading: grams of air entering per second, as an OBD2 reader shows it
    double PedalPosition);           // what the driver is asking for: 0 = foot off, 1 = floored

// Everything the ECU decides in one tick. The Vehicle applies these to the components.
// AllowRun + Message come from the run check, SparkOn goes to the ignition,
// IdleThrottleOpening and FuelPerCycleG to the engine.
public record EcuOutputs(
    bool AllowRun,
    bool SparkOn,
    double IdleThrottleOpening,      // extra throttle plate opening the ECU adds on top of the pedal to hold idle (0–1)
    double FuelPerCycleG,
    double SparkAdvanceDeg,
    string? Message);

// Every control decision lives here. The ECU holds NO references to other systems:
// the Vehicle passes in the facts as parameters, and the ECU returns decisions.
// It can keep its own internal state (idle wobble timer, rev limiter flag).
public class EngineControlUnit
{
    public const double TargetIdleRpm = 700;
    private const double BaseIdleThrottleOpening = 0.015;  // best first guess at the idle plate opening: 1.5%, where torque balances friction + pumping
    private const double IdleKp = 0.000016;                // plate opening per RPM of error (100 RPM low → +0.16% ≈ +3.6 Nm)
    private const double IdleKi = 0.00002;                 // plate opening per RPM·second of accumulated error
    public const double MaxIdleThrottleOpening = 0.06;     // the most the ECU will open the plate for idle (6%)
    private const double IdlePedalFreeze = 0.02;           // above this pedal, the idle controller holds still
    private const double DashpotBandRpm = 100;             // more than this above the idle target, the plate won't close below its idle position
    private const double StallRpm = 250;                   // below this while running (after the grace period), the engine has stalled
    private const double StartGracePeriodS = 1.0;          // no stall check for this long after starting
    private const double IdleVariation = 0.02;        // idle wobbles by ± this fraction of target idle
    private const double IdleWobbleIntervalS = 1.0;   // seconds between new random idle targets
    private const double IdleWobbleRate = 2.0;        // how quickly the idle target drifts toward its latest random value
    private const double RevLimiterRestoreGap = 150;  // spark returns this far below max RPM
    private const double LowFuelPercentage = 5;       // low fuel warning below this % of the tank
    private const double MinRpmForFuel = 50;          // below this the MAF reading means nothing, so no fuel

    // Idle control state
    private double _idleTargetRpm = TargetIdleRpm;            // latest random target
    private double _smoothedIdleTargetRpm = TargetIdleRpm;    // eased toward it: what the controller aims at
    private double _timeUntilNewIdleTarget;
    private double _rpmErrorIntegral;                         // accumulated RPM error (RPM·s)
    private double _idleThrottleOpening = BaseIdleThrottleOpening;

    // Stall check state: how long the engine has been running on its own (not cranking)
    private double _timeRunningS;

    // Rev limiter state: true while spark is being cut
    private bool _limiterActive;

    // The fuel this ECU is calibrated for. Data only, not a reference to the fuel system.
    private readonly FuelProperties _fuel;

    public EngineControlUnit(FuelProperties fuel)
    {
        _fuel = fuel;
    }


    // One tick of the ECU: read the inputs, make every decision, return them together.
    public EcuOutputs Update(double dt, EcuInputs inputs)
    {
        RunCheckResult check = CheckCanRun(inputs.BatteryConnected, inputs.FuelLevelPercentage);
        if (check.Allowed && HasStalled(dt, inputs))
            check = new RunCheckResult(false, "engine stalled");
        bool sparkOn = DecideSpark(inputs);
        double idleThrottleOpening = UpdateIdleThrottle(dt, inputs);
        double fuelPerCycle = DecideFuel(inputs, sparkOn);

        return new EcuOutputs(check.Allowed, sparkOn, idleThrottleOpening, fuelPerCycle, 20, check.Message);
    }

    // Fuel (#5): air per cycle from the MAF reading, ÷ target air-fuel ratio.
    // MAF reads per second, so divide by cycles per second (RPM / 120) to get per cycle.
    private double DecideFuel(EcuInputs inputs, bool sparkOn)
    {
        if (!sparkOn)
            return 0; // fuel cut with the spark: engine off, no battery, or rev limiter

        if (inputs.Rpm < MinRpmForFuel)
            return 0; // stopped or barely turning: no meaningful reading, and avoids dividing by 0

        double airPerCycleG = inputs.MafGPerS / (inputs.Rpm / 120);
        return airPerCycleG / TargetAfr();
    }

    // The air-fuel ratio to run at. Always stoichiometric for now;
    // richer at full throttle can come later by changing only this.
    private double TargetAfr()
    {
        return _fuel.StoichiometricAfr;
    }

    // Run checks: used at start AND every tick, so starting and stalling follow the same rules.
    // Each ticket adds one (#2 battery, #4 fuel)
    private RunCheckResult CheckCanRun(bool batteryConnected, double fuelLevelPercentage)
    {
        if (!batteryConnected)
            return new RunCheckResult(false, "battery not connected");
        if (fuelLevelPercentage <= 0)
            return new RunCheckResult(false, "fuel tank empty");
        if (fuelLevelPercentage < LowFuelPercentage)
            return new RunCheckResult(true, "fuel level low");

        return new RunCheckResult(true, null);
    }

    // Stall (#32): with no idle floor, RPM can fall to 0. Running (not cranking) below StallRpm means it has stalled.
    // Skipped for StartGracePeriodS after starting, so the engine has time to pick up from cranking speed.
    private bool HasStalled(double dt, EcuInputs inputs)
    {
        if (!inputs.EngineRunning || inputs.Cranking)
        {
            _timeRunningS = 0;
            return false;
        }

        _timeRunningS += dt;
        return _timeRunningS > StartGracePeriodS && inputs.Rpm < StallRpm;
    }

    // Status message for the display. The Vehicle passes in whether the engine is running.
    public string EngineStatus(bool engineRunning)
    {
        return engineRunning ? "Engine running" : "Engine off";
    }


    // Spark (#6): needs battery power, an engine that is running or cranking, and the rev limiter's OK.
    // The limiter is updated every tick regardless, so its state tracks RPM even while spark is off.
    private bool DecideSpark(EcuInputs inputs)
    {
        bool limiterOk = UpdateRevLimiter(inputs.Rpm, inputs.MaxRpm);
        return inputs.BatteryConnected && (inputs.EngineRunning || inputs.Cranking) && limiterOk;
    }

    // Rev limiter (#6): cut spark at max RPM, restore it once RPM falls below max by the gap.
    // The gap stops spark flickering on and off every tick. Returns whether spark is allowed.
    private bool UpdateRevLimiter(double rpm, double maxRpm)
    {
        if (rpm >= maxRpm)
            _limiterActive = true;
        else if (rpm < maxRpm - RevLimiterRestoreGap)
            _limiterActive = false;

        return !_limiterActive;
    }

    // Idle control (#32): the ECU holds the drive-by-wire throttle plate slightly open to keep idle on a wobbling ~700 RPM target.
    // Returns how much plate opening to add on top of the pedal this tick (0 to MaxIdleThrottleOpening).
    private double UpdateIdleThrottle(double dt, EcuInputs inputs)
    {
        // 1. Wobble: pick a new random target every IdleWobbleIntervalS and ease toward it, so idle sounds uneven
        _timeUntilNewIdleTarget -= dt;
        if (_timeUntilNewIdleTarget <= 0)
        {
            int idleRange = (int)(TargetIdleRpm * IdleVariation);
            _idleTargetRpm = TargetIdleRpm + Random.Shared.Next(-idleRange, idleRange + 1);
            _timeUntilNewIdleTarget = IdleWobbleIntervalS;
        }
        _smoothedIdleTargetRpm += (_idleTargetRpm - _smoothedIdleTargetRpm) * IdleWobbleRate * dt;

        // 2. Engine off: reset to the first guess, so a restart doesn't begin from whatever the controller wound up to
        if (!inputs.EngineRunning)
        {
            _rpmErrorIntegral = 0;
            _idleThrottleOpening = BaseIdleThrottleOpening;
            return _idleThrottleOpening;
        }

        // 3. Pedal pressed: freeze the controller, so it isn't wound right down when the driver lets go
        if (inputs.PedalPosition > IdlePedalFreeze)
            return _idleThrottleOpening;

        // 4. PI controller: open more when RPM is below target, both straight away (P) and the longer it lasts (I)
        double rpmErrorRpm = _smoothedIdleTargetRpm - inputs.Rpm;      // positive = idling too low
        double newIntegral = _rpmErrorIntegral + rpmErrorRpm * dt;
        double opening = BaseIdleThrottleOpening + IdleKp * rpmErrorRpm + IdleKi * newIntegral;

        // 5. Dashpot: coasting down well above idle (after lifting off), don't let the plate close below its idle position.
        //    Snapping it shut would drop the plenum into deep vacuum, and the pumping loss would brake the engine straight
        //    through idle (down to ~540 RPM after a full-throttle blip). Hold the integral too, so it doesn't wind down.
        if (inputs.Rpm > _smoothedIdleTargetRpm + DashpotBandRpm)
        {
            opening = Math.Max(opening, BaseIdleThrottleOpening);
            newIntegral = _rpmErrorIntegral;
        }

        // 6. Anti-windup: only keep the new integral if the opening isn't pinned at a limit
        if (opening >= 0 && opening <= MaxIdleThrottleOpening)
            _rpmErrorIntegral = newIntegral;

        _idleThrottleOpening = Math.Clamp(opening, 0, MaxIdleThrottleOpening);
        return _idleThrottleOpening;
    }
}
