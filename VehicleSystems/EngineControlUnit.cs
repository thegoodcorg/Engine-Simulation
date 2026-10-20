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
    double MafKgPerS);           // mass airflow (MAF) reading: air entering per second

// Everything the ECU decides in one tick. The Vehicle applies these to the components.
// AllowRun + Message come from the run check, SparkOn goes to the ignition,
// IdleFloorRpm and FuelPerCycleKg to the engine.
public record EcuOutputs(bool AllowRun, bool SparkOn, double IdleFloorRpm, double FuelPerCycleKg, string? Message);

// Every control decision lives here. The ECU holds NO references to other systems:
// the Vehicle passes in the facts as parameters, and the ECU returns decisions.
// It can keep its own internal state (idle wobble timer, rev limiter flag).
public class EngineControlUnit
{
    public const double TargetIdleRpm = 700;
    private const double IdleVariation = 0.02;        // idle wobbles by ± this fraction of target idle
    private const double IdleWobbleIntervalS = 1.0;   // seconds between new random idle targets
    private const double IdleWobbleRate = 2.0;        // how quickly the floor drifts toward its target
    private const double RevLimiterRestoreGap = 150;  // spark returns this far below max RPM
    private const double LowFuelPercentage = 5;       // low fuel warning below this % of the tank
    private const double MinRpmForFuel = 50;          // below this the MAF reading means nothing, so no fuel

    // Idle floor state
    private double _idleFloorRpm = TargetIdleRpm;
    private double _idleTargetRpm = TargetIdleRpm;
    private double _timeUntilNewIdleTarget;

    // Rev limiter state: true while spark is being cut
    private bool _limiterActive;

    // The fuel this ECU is calibrated for. Data only, not a reference to the fuel system.
    private readonly FuelProperties _fuel;

    // fuel defaults to gasoline when not given
    public EngineControlUnit(FuelProperties fuel)
    {
        _fuel = fuel;
    }


    // One tick of the ECU: read the inputs, make every decision, return them together.
    public EcuOutputs Update(double dt, EcuInputs inputs)
    {
        RunCheckResult check = CheckCanRun(inputs.BatteryConnected, inputs.FuelLevelPercentage);
        bool sparkOn = DecideSpark(inputs);
        double idleFloor = UpdateIdleFloor(dt);
        double fuelPerCycle = DecideFuel(inputs);

        return new EcuOutputs(check.Allowed, sparkOn, idleFloor, fuelPerCycle, check.Message);
    }

    // Fuel (#5): air per cycle from the MAF reading, ÷ target air-fuel ratio.
    // MAF reads per second, so divide by cycles per second (RPM / 120) to get per cycle.
    private double DecideFuel(EcuInputs inputs)
    {
        if (inputs.Rpm < MinRpmForFuel)
            return 0; // stopped or barely turning: no meaningful reading, and avoids dividing by 0

        double airPerCycleKg = inputs.MafKgPerS / (inputs.Rpm / 120);
        return airPerCycleKg / TargetAfr();
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

    // Idle floor (moved here from Engine): picks a new random target every IdleWobbleIntervalS
    // seconds and eases the floor toward it. Returns the floor for this tick.
    private double UpdateIdleFloor(double dt)
    {
        _timeUntilNewIdleTarget -= dt;
        if (_timeUntilNewIdleTarget <= 0)
        {
            int idleRange = (int)(TargetIdleRpm * IdleVariation);
            _idleTargetRpm = TargetIdleRpm + Random.Shared.Next(-idleRange, idleRange + 1);
            _timeUntilNewIdleTarget = IdleWobbleIntervalS;
        }
        _idleFloorRpm += (_idleTargetRpm - _idleFloorRpm) * IdleWobbleRate * dt;
        return _idleFloorRpm;
    }
}
