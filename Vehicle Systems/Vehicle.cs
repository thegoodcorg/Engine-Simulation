using EngineSim.Constants;

namespace EngineSim;

public class Vehicle
{
    // Components are private: Program.cs talks to the Vehicle, never to a component directly
    private readonly AmbientConditions _ambientConditions;
    private readonly Engine _engine;
    private readonly EngineControlUnit _ecu;
    private readonly FuelSystem _fuelSystem;
    // private readonly Battery _battery;
    // private readonly Ignition _ignition;


    public Vehicle(double bore, double stroke, int cylinders, AmbientConditions ambientConditions)
    {
        // One shared AmbientConditions, handed to anything that needs the outside air
        _ambientConditions = ambientConditions;

        _engine = new Engine(bore, stroke, cylinders, _ambientConditions);
        _ecu = new EngineControlUnit(_engine.TargetIdleRpm);
        _fuelSystem = new FuelSystem(tankSizeL: 80);
        // _battery = new Battery();
        // _ignition = new Ignition();
    }

    // ---- Values for Program.cs to display (read-only) ----
    public double Rpm => _engine.EngineSpeed;
    public double ThrottlePosition => _engine.ThrottlePosition;
    public double EngineTorqueNm => _engine.TorqueNm;
    public double FuelLevelL => _fuelSystem.FuelLevelL;
    public bool IsRunning => _engine.IsRunning;
    public string EngineStatus => _ecu.EngineStatus(_engine.IsRunning);
    // public bool SparkOn => _ignition.SparkOn;

    // Engine specs for the startup summary
    public double BoreMm => _engine.BoreMm;
    public double StrokeLengthMm => _engine.StrokeLengthMm;
    public int CylinderCount => _engine.CylinderCount;
    public double DisplacementLiters => _engine.DisplacementLiters;
    public double MaxRpm => _engine.MaxRpm;
    public double PeakTorqueNm => _engine.TorqueNm;

    public void Update(double dt)
    {
        double rpm = _engine.EngineSpeed;
        // 1 read current values fuel level, battery, etc
        // 2 pass things to the ecu and decide what to do
        bool sparkOn = _ecu.UpdateRevLimiter(rpm, MaxRpm);
        double idleFloor = _ecu.UpdateIdleFloor(dt, _engine.TargetIdleRpm);

        // 3 update engine as usual
        _engine.Update(dt, sparkOn, idleFloor);
    }

    // Start key: turns the engine off if running, otherwise asks the ECU and starts it if allowed
    public void StartVehicle()
    {
        if (_engine.IsRunning)
        {
            _engine.Stall();
            return;
        }

        // No Battery class yet (#2), so the battery always counts as connected
        StartCheckResult check = _ecu.CheckCanStart(batteryConnected: true, _fuelSystem.FuelLevelL);
        if (check.Allowed)
            _engine.StartEngine();
    }


    // Open (+) or close (-) the throttle by the given amount; the setter keeps it between 0 and 1
    public void ThrottleControl(double amount)
    {
        _engine.ThrottlePosition += amount;
    }
}