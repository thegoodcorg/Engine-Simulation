using EngineSim.Constants;
using EngineSim.Enums;
using EngineSim.VehicleSystems.Body;
using EngineSim.VehicleSystems.Driveline;
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
    private readonly Drivetrain _drivetrain;

    // Driver and road inputs (#37)
    public double BrakePedal { get; set; }      // 0 = off, 1 = full braking
    public double GradePercent { get; set; }    // hill steepness: 5 = climbing 5 m per 100 m, negative = downhill. 0 = flat

    // Keyboard controls (#39). A key is only up or down, so the clutch and brake ramp over a short time instead of
    // jumping: releasing the clutch slips it for a moment, which is what makes pulling away possible from a keyboard.
    public const double ClutchEngageTimeS = 0.6;   // clutch pedal released: fully engaged after this long
    public const double BrakeApplyTimeS = 0.3;     // brake key held: full pressure after this long
    public const double MaxGradePercent = 15;      // steepest hill the grade keys allow, up or down

    private bool _clutchPedalHeld;
    private bool _brakeHeld;
    // The ramps only take over once the keys are used, so code that sets ClutchEngagement or BrakePedal directly
    // (tests, the dyno) isn't overridden every tick
    private bool _clutchPedalInUse;
    private bool _brakeKeyInUse;

    // A short message for the driver, e.g. a refused shift. Cleared by the next shift that works.
    public string? DriverMessage { get; private set; }

    // Drivetrain (#38)
    public double ClutchEngagement                // 0 = clutch pedal down, 1 = pedal fully out
    {
        get => _drivetrain.Clutch.Engagement;
        set => _drivetrain.Clutch.Engagement = value;
    }
    public int Gear => _drivetrain.Gearbox.Gear;              // 0 = neutral, 1–9
    public ClutchState ClutchState => _drivetrain.Clutch.State;

    // Change gear. Refused (false) unless the clutch pedal is all the way down.
    public bool ShiftTo(int gear)
    {
        bool shifted = _drivetrain.Gearbox.TryShift(gear, clutchDisengaged: ClutchEngagement == 0);
        DriverMessage = shifted ? null : "press clutch to shift";
        return shifted;
    }

    // Up one gear (neutral → 1st). Does nothing in top gear.
    public bool ShiftUp() => Gear < Gearbox.TopGear && ShiftTo(Gear + 1);

    // Down one gear. Does nothing in 1st or neutral: neutral is its own key.
    public bool ShiftDown() => Gear > 1 && ShiftTo(Gear - 1);

    public bool SelectNeutral() => ShiftTo(0);

    // Clutch pedal: held = pedal down (clutch fully in straight away); released = engages over ClutchEngageTimeS
    public void SetClutchPedalHeld(bool held)
    {
        _clutchPedalHeld = held;
        _clutchPedalInUse = true;
    }

    // Brake key: held = pressure builds over BrakeApplyTimeS; released = off straight away
    public void SetBrakeHeld(bool held)
    {
        _brakeHeld = held;
        _brakeKeyInUse = true;
    }

    // Grade keys: steeper or shallower by this many percent, kept within ±MaxGradePercent
    public void ChangeGrade(double deltaPercent)
    {
        GradePercent = Math.Clamp(GradePercent + deltaPercent, -MaxGradePercent, MaxGradePercent);
    }

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
        _drivetrain = new Drivetrain();
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
    public double SpeedMph => _body.SpeedMs * 2.23694;
    public double AccelerationMs2 => _body.AccelerationMs2;

    // Engine specs for the startup summary
    public double BoreMm => _engine.Geometry.BoreMm;
    public double StrokeLengthMm => _engine.Geometry.StrokeLengthMm;
    public double RodLengthMm => _engine.Geometry.RodLengthMm;
    public double CompressionRatio => _engine.Geometry.CompressionRatio;
    public int CylinderCount => _engine.CylinderCount;
    public double DisplacementLiters => _engine.DisplacementLiters;
    public double MaxRpm => _engine.MaxRpm;
    public double RealTimeFactor => _engine.RealTimeFactor; // how fast the sim ran last tick compared with real time (#29)

    public void Update(double dt)
    {
        UpdatePedals(dt);

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


        double roadLoadN = _body.RollingResistanceN + _body.AeroDragN + _body.GradeForceN + _body.BrakeForceN;

        var coupling = _drivetrain.Update(dt, _engine.EngineSpeed, _engine.NetTorque, _engine.Inertia, _body.SpeedMs, roadLoadN, VehicleBody.massKg);
        // Spark timing from the ECU; null while the spark is off (key off, rev limiter, stall), so nothing is injected or burned.
        // The starter turns the crank while it's engaged, so the cylinders fill and fire before it lets go.
        _engine.SparkAdvanceDeg = _ignitionSystem.SparkOn ? ecu.SparkAdvanceDeg : null;
        _engine.StarterEngaged = _starter.StarterEngaged;
        _engine.Update(dt, ecu.IdleThrottleOpening, coupling.LoadTorqueNm, coupling.ExtraInertiaKgM2);

        // Drain what the engine used: grams ÷ 1000 = kg, then ÷ density (kg/L) = liters
        _fuelSystem.Drain(_engine.FuelBurnedG / 1000 / _fuelSystem.Fuel.DensityKgPerL);

        _body.Update(dt, coupling.DriveForceN, BrakePedal, GradePercent, _engine.OutsideAirDensityKgM3);

        if (coupling.State == ClutchState.Locked)
        {
            _body.SetSpeedMs(_drivetrain.SpeedAtRpmMs(Rpm));
        }
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


    // Clutch and brake ramps (#39), scaled by dt so they feel the same at any tick rate
    private void UpdatePedals(double dt)
    {
        if (_clutchPedalInUse)
            ClutchEngagement = _clutchPedalHeld ? 0 : Math.Min(1, ClutchEngagement + dt / ClutchEngageTimeS);

        if (_brakeKeyInUse)
            BrakePedal = _brakeHeld ? Math.Min(1, BrakePedal + dt / BrakeApplyTimeS) : 0;
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
