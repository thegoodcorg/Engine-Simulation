using EngineSim.VehicleSystems.Body;
using EngineSim.VehicleSystems.Electrical;

namespace EngineSim;

public class Vehicle
{
    // Components are private: Program.cs talks to the Vehicle, never to a component directly
    private readonly FuelSystem _fuelSystem;
    private readonly Engine _engine;
    private readonly EngineControlUnit _ecu;
    // electrical components
    private readonly Battery _battery;
    private readonly Starter _starter;
    private readonly IgnitionSystem _ignitionSystem;
    private readonly VehicleBody _body;

    // Driver and road inputs (#37). The controls ticket sets these from the keyboard.
    public double BrakePedal { get; set; }      // 0 = off, 1 = full braking
    public double GradePercent { get; set; }    // hill steepness: 5 = climbing 5 m per 100 m, negative = downhill. 0 = flat

    private bool _startKeyHeld;         // set by Program.cs every tick
    private string? _diagnosticMessage; // latest ECU message: a refusal, stall reason or warning, or null
    // The engine is built outside and handed in, so its specs and air conditions are chosen by the caller
    public Vehicle(Engine engine)
    {
        _fuelSystem = new FuelSystem(80);
        _engine = engine;
        _ecu = new EngineControlUnit(_fuelSystem.Fuel);
        _battery = new Battery();
        _starter = new Starter();
        _ignitionSystem = new IgnitionSystem();
        _body = new VehicleBody();
    }

    // ---- Values for Program.cs to display (read-only) ----
    public double Rpm => _engine.EngineSpeed;
    public double TorqueNm => _engine.NetTorque;              // torque at the crank, after friction (negative while the idle floor holds RPM up)
    public double MafGPerS => _engine.MafGPerS;           // mass airflow, as an OBD2 reader shows it
    public double ThrottlePosition => _engine.ThrottlePosition;
    public double ThrottlePlateOpening => _engine.ThrottlePlateOpening; // pedal + the ECU's idle opening: where the plate actually is
    public double FuelLevelL => _fuelSystem.FuelLevelL;
    public bool IsRunning => _engine.IsRunning;
    public bool OutOfFuel => _fuelSystem.FuelLevelL <= 0; // Program.cs ends the sim on this
    // Engine state, plus the ECU's message when there is one, e.g. "Engine running: fuel level low"
    public double ManifoldPressureKpa => _engine.ManifoldPressureKpa;
    public string EngineStatus
    {
        get
        {
            string status = _starter.StarterEngaged ? "Engine cranking" : _ecu.EngineStatus(_engine.IsRunning);
            return _diagnosticMessage is null ? status : $"{status}: {_diagnosticMessage}";
        }
    }
    public double BatteryVoltage => _battery.IsBatteryConnected ? _battery.BatteryChargeV : 0;
    public bool SparkOn => _ignitionSystem.SparkOn;

    // Road (#37)
    public double SpeedMs => _body.SpeedMs;
    public double SpeedKmh => _body.SpeedKmh;
    public double AccelerationMs2 => _body.AccelerationMs2;

    // Engine specs for the startup summary
    public double BoreMm => _engine.Geometry.BoreMm;
    public double StrokeLengthMm => _engine.Geometry.StrokeLengthMm;
    public double RodLengthMm => _engine.Geometry.RodLengthMm;
    public double CompressionRatio => _engine.Geometry.CompressionRatio;
    public int CylinderCount => _engine.CylinderCount;
    public double DisplacementLiters => _engine.DisplacementLiters;
    public double MaxRpm => _engine.MaxRpm;
    public double PeakTorqueNm => _engine.MaxBrakeTorqueNm; // at the crank, after friction

    public void Update(double dt)
    {
        // 1 read the sensors, 2 let the ECU decide everything, 3 apply its decisions
        EcuOutputs ecu = _ecu.Update(dt, ReadSensors());
        _ignitionSystem.SetSpark(ecu.SparkOn);

        if (_engine.IsRunning)
        {
            if (!ecu.AllowRun)
                _engine.Stall();

            // While running, track the ECU live: warnings appear and clear as conditions change.
            // While off, leave the message alone so a refused start's reason stays up.
            _diagnosticMessage = ecu.Message;
        }
        else
        {
            UpdateStarter(dt, ecu);
        }

        _engine.Update(dt, _ignitionSystem.SparkOn, ecu.IdleThrottleOpening, ecu.FuelPerCycleG);

        // Drain what the engine used: grams ÷ 1000 = kg, then ÷ density (kg/L) = liters
        _fuelSystem.Drain(_engine.FuelBurnedG / 1000 / _fuelSystem.Fuel.DensityKgPerL);

        // The truck on the road. No drivetrain yet, so no drive force: it only coasts, brakes and rolls on hills
        _body.Update(dt, driveForceN: 0, BrakePedal, GradePercent, _engine.OutsideAirDensityKgM3);
    }

    // Snapshot of everything the ECU reads this tick
    private EcuInputs ReadSensors()
    {
        return new EcuInputs(
            Rpm: _engine.EngineSpeed,
            MaxRpm: _engine.MaxRpm,
            BatteryConnected: _battery.IsBatteryConnected,
            FuelLevelPercentage: _fuelSystem.FuelLevelPercentage,
            EngineRunning: _engine.IsRunning,
            Cranking: _starter.StarterEngaged,
            MafGPerS: _engine.MafGPerS,
            PedalPosition: _engine.ThrottlePosition);
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
    private void UpdateStarter(double dt, EcuOutputs ecu)
    {
        if (!_startKeyHeld)
        {
            _starter.Release();
            return;
        }

        if (!ecu.AllowRun)
        {
            _diagnosticMessage = ecu.Message;
            _starter.Release();
            return;
        }

        _starter.HasBatteryPower = _battery.IsBatteryConnected;
        _starter.TurnEngine();
        _starter.Update(dt);

        if (_starter.EngagedTimeSec >= Engine.CrankTimeSeconds)
        {
            _diagnosticMessage = ecu.Message; // null, or a warning like low fuel
            _engine.StartEngine();
            _starter.Release();
        }
    }


    // Open (+) or close (-) the throttle by the given amount; the setter keeps it between 0 (released) and 1
    public void ThrottleControl(double amount)
    {
        _engine.ThrottlePosition += amount;
    }
    private void ClearDiagnosticMessage()
    {
        _diagnosticMessage = null;
    }
}
