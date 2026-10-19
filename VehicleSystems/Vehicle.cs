using EngineSim.Constants;
using EngineSim.Electrical;

namespace EngineSim;

public class Vehicle
{
    // Components are private: Program.cs talks to the Vehicle, never to a component directly
    private readonly AmbientConditions _ambientConditions;
    private readonly Engine _engine;
    private readonly EngineControlUnit _ecu;
    private readonly FuelSystem _fuelSystem;

    // electrical components
    private readonly Battery _battery;

    private readonly Starter _starter;
    // private readonly Ignition _ignition;

    private bool _startKeyHeld;         // set by Program.cs every tick
    private string? _diagnosticMessage; // latest ECU message: a refusal, stall reason or warning, or null
    public Vehicle(double bore, double stroke, int cylinders, AmbientConditions ambientConditions)
    {
        // One shared AmbientConditions, handed to anything that needs the outside air
        _ambientConditions = ambientConditions;

        _engine = new Engine(bore, stroke, cylinders, _ambientConditions);
        _ecu = new EngineControlUnit(_engine.TargetIdleRpm);
        _fuelSystem = new FuelSystem(80);
        _battery = new Battery();
        _starter = new Starter();
        // _ignition = new Ignition();
    }

    // ---- Values for Program.cs to display (read-only) ----
    public double Rpm => _engine.EngineSpeed;
    public double ThrottlePosition => _engine.ThrottlePosition;
    public double FuelLevelL => _fuelSystem.FuelLevelL;
    public bool IsRunning => _engine.IsRunning;
    public bool OutOfFuel => _fuelSystem.FuelLevelL <= 0; // Program.cs ends the sim on this
    // Engine state, plus the ECU's message when there is one, e.g. "Engine running: fuel level low"
    public string EngineStatus
    {
        get
        {
            string status = _starter.StarterEngaged ? "Engine cranking" : _ecu.EngineStatus(_engine.IsRunning);
            return _diagnosticMessage is null ? status : $"{status}: {_diagnosticMessage}";
        }
    }
    public double BatteryVoltage => _battery.IsBatteryConnected ? _battery.BatteryChargeV : 0;
    // public bool SparkOn => _ignition.SparkOn;

    // Engine specs for the startup summary
    public double BoreMm => _engine.BoreMm;
    public double StrokeLengthMm => _engine.StrokeLengthMm;
    public int CylinderCount => _engine.CylinderCount;
    public double DisplacementLiters => _engine.DisplacementLiters;
    public double MaxRpm => _engine.MaxRpm;
    public double PeakTorqueNm => _engine.MaxTorqueNm;

    public void Update(double dt)
    {
        double rpm = _engine.EngineSpeed;
        // 1 read current values fuel level, battery, etc
        // 2 pass things to the ecu and decide what to do
        RunCheckResult check = _ecu.CheckCanRun(_battery.IsBatteryConnected, _fuelSystem.FuelLevelPercentage);
        if (_engine.IsRunning)
        {
            if (!check.Allowed)
                _engine.Stall();

            // While running, track the ECU live: warnings appear and clear as conditions change.
            // While off, leave the message alone so a refused start's reason stays up.
            _diagnosticMessage = check.Message;
        }
        else
        {
            UpdateStarter(dt, check);
        }
        bool sparkOn = _ecu.UpdateRevLimiter(rpm, MaxRpm);
        double idleFloor = _ecu.UpdateIdleFloor(dt, _engine.TargetIdleRpm);

        // 3 update engine as usual
        _engine.Update(dt, sparkOn, idleFloor);
    }

    public void SwitchBattery()
    {
        _battery.SwitchBattery();
        ClearDiagnosticMessage(); // the old failure may no longer apply
    }

    // Start key: Program.cs reports every tick whether it's held down
    public void SetStartKeyHeld(bool held)
    {
        _startKeyHeld = held;
    }

    // Stop key: turns the engine off. Does nothing if it's already off.
    public void ShutdownEngine()
    {
        _engine.Shutdown();
    }

    // While the engine is off and the start key is held, the starter turns the engine.
    // Held for Engine.CrankTimeSeconds → engine starts. Let go early → the timer resets.
    // The ECU check runs every tick, so a refusal shows straight away and a fault mid-crank stops it.
    private void UpdateStarter(double dt, RunCheckResult check)
    {
        if (!_startKeyHeld)
        {
            _starter.Release();
            return;
        }

        if (!check.Allowed)
        {
            _diagnosticMessage = check.Message;
            _starter.Release();
            return;
        }

        _starter.HasBatteryPower = _battery.IsBatteryConnected;
        _starter.TurnEngine();
        _starter.Update(dt);

        if (_starter.EngagedTimeSec >= Engine.CrankTimeSeconds)
        {
            _diagnosticMessage = check.Message; // null, or a warning like low fuel
            _engine.StartEngine();
            _starter.Release();
        }
    }


    // Open (+) or close (-) the throttle by the given amount; the setter keeps it between MinThrottle (6%) and 1
    public void ThrottleControl(double amount)
    {
        _engine.ThrottlePosition += amount;
    }
    private void ClearDiagnosticMessage()
    {
        _diagnosticMessage = null;
    }
}
