namespace EngineSim;

// The answer to "may the engine start?": allowed or not, and why not
public record StartCheckResult(bool Allowed, string Message);

// Every control decision lives here. The ECU holds NO references to other systems:
// the Vehicle passes in the facts as parameters, and the ECU returns decisions.
// It can keep its own internal state (idle wobble timer, rev limiter flag).
public class EngineControlUnit
{
    private const double IdleVariation = 0.02;        // idle wobbles by ± this fraction of target idle
    private const double IdleWobbleIntervalS = 1.0;   // seconds between new random idle targets
    private const double IdleWobbleRate = 2.0;        // how quickly the floor drifts toward its target
    private const double RevLimiterRestoreGap = 150;  // spark returns this far below max RPM

    // Idle floor state
    private double _idleFloorRpm;
    private double _idleTargetRpm;
    private double _timeUntilNewIdleTarget;

    // Rev limiter state: true while spark is being cut
    private bool _limiterActive;

    public double IdleFloorRpm => _idleFloorRpm;



    public EngineControlUnit(double targetIdleRpm)
    {
        _idleFloorRpm = targetIdleRpm;
        _idleTargetRpm = targetIdleRpm;
    }

    // Start checks: each ticket adds one (#2 battery, #4 fuel)
    public StartCheckResult CheckCanStart(bool batteryConnected, double fuelLevelL)
    {
        if (!batteryConnected)
            return new StartCheckResult(false, "battery not connected");

        if (fuelLevelL <= 0)
            return new StartCheckResult(false, "fuel tank empty");

        return new StartCheckResult(true, "engine started");
    }

    // Status message for the display. The Vehicle passes in whether the engine is running.
    public string EngineStatus(bool engineRunning)
    {
        return engineRunning ? "Engine running" : "Engine off";
    }

    // Stall check (#4): no fuel, no running
    public bool ShouldStall(double fuelLevelL)
    {
        return fuelLevelL <= 0;
    }

    // Rev limiter (#6): cut spark at max RPM, restore it once RPM falls below max by the gap.
    // The gap stops spark flickering on and off every tick. Returns whether spark is allowed.
    public bool UpdateRevLimiter(double rpm, double maxRpm, bool isBatteryConnected)
    {
        if (!isBatteryConnected) return false;

        if (rpm >= maxRpm)
            _limiterActive = true;
        else if (rpm < maxRpm - RevLimiterRestoreGap)
            _limiterActive = false;

        return !_limiterActive;
    }

    // Idle floor (moved here from Engine): picks a new random target every IdleWobbleIntervalS
    // seconds and eases the floor toward it. Returns the floor for this tick.
    public double UpdateIdleFloor(double dt, double targetIdleRpm)
    {
        _timeUntilNewIdleTarget -= dt;
        if (_timeUntilNewIdleTarget <= 0)
        {
            int idleRange = (int)(targetIdleRpm * IdleVariation);
            _idleTargetRpm = targetIdleRpm + Random.Shared.Next(-idleRange, idleRange + 1);
            _timeUntilNewIdleTarget = IdleWobbleIntervalS;
        }
        _idleFloorRpm += (_idleTargetRpm - _idleFloorRpm) * IdleWobbleRate * dt;
        return _idleFloorRpm;
    }
}
