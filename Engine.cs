using EngineSim.Constants;

namespace EngineSim;

public class Engine
{
    // Tuning constants
    private const double PistonSpeedLimitMs = 20;        // max mean piston speed, sets the redline
    private const double IdleVariation = 0.02;           // idle wobbles by ± this fraction of TargetIdleRpm
    private const double IdleWobbleIntervalS = 1.0;      // seconds between new random idle targets
    private const double IdleWobbleRate = 2.0;           // how quickly the idle floor drifts toward its target
    private const double BmepBar = 12.5;                 // brake mean effective pressure, used for torque
    private const double FmepBar = 1.2;                    // Friction mean effective pressure, used for torque
    private const double InertiaPerLiter = 0.05;            // kg·m² per liter of displacement

    public double BoreMm { get; }  // Diameter of the cylinder
    public double StrokeLengthMm { get; } // distance the piston moves up/down
    public int CylinderCount { get; }
    public double CylinderVolumeLiters { get; }
    public double DisplacementLiters { get; }
    public double MaxRpm { get; }
    public double TargetIdleRpm { get; } = 700;
    public double EngineSpeed { get; private set; }
    public double TorqueNm { get; set; } // BMEP(Pa) * dsiplacement(m^3)/(4pi)
    public double FrictionTorque { get; } // FMEP * displacement / (4pi)
    public double NetTorque { get; private set; }
    public double Inertia { get; }

    // Idle floor drifts toward a random target near TargetIdleRpm, so idle wobbles smoothly
    private double _idleFloorRpm;
    private double _idleTargetRpm;
    private double _timeUntilNewIdleTarget;

    // 0.0 = closed, 1.0 = wide open. Values outside that range get clamped.
    private double _throttlePosition;
    public double ThrottlePosition
    {
        get => _throttlePosition;
        set => _throttlePosition = Math.Clamp(value, 0.0, 1.0);
    }

    public Engine(double bore, double stroke, int cylinders, AmbientConditions ambientConditions)
    {
        BoreMm = bore;
        StrokeLengthMm = stroke;
        CylinderCount = cylinders;
        CylinderVolumeLiters = CalculateCylinderVolumeLiters();
        DisplacementLiters = TotalDisplacement();
        MaxRpm = CalculateRedline();
        EngineSpeed = TargetIdleRpm;
        _idleFloorRpm = TargetIdleRpm;
        _idleTargetRpm = TargetIdleRpm;
        TorqueNm = PressureToTorqueNm(BmepBar);
        FrictionTorque = PressureToTorqueNm(FmepBar);
        Inertia = InertiaPerLiter * DisplacementLiters;
    }

    // Advance the engine by dt seconds
    public void Update(double dt)
    {
        // 1. Torque at the current throttle, minus what friction takes away
        NetTorque = TorqueNm * ThrottlePosition - FrictionTorque;

        // 2. Rotational version of F = ma: angular acceleration = torque / inertia (rad/s²)
        double angularAcceleration = NetTorque / Inertia;

        // 3. Convert rad/s² to RPM per second, then scale by the time step
        EngineSpeed += angularAcceleration * 60 / (2 * Math.PI) * dt;

        // 4. Idle floor (moves to the ECU later) and redline ceiling (until the rev limiter)
        UpdateIdleFloor(dt);
        EngineSpeed = Math.Clamp(EngineSpeed, _idleFloorRpm, MaxRpm);
    }

    // Volume swept by one piston: π/4 × bore² × stroke, converted from mm³ to liters
    private double CalculateCylinderVolumeLiters()
    {
        return Math.PI / 4 * Math.Pow(BoreMm, 2) * StrokeLengthMm / 1_000_000;
    }

    private double TotalDisplacement()
    {
        return CalculateCylinderVolumeLiters() * CylinderCount;
    }

    private double CalculateRedline()
    {
        double maxRpm = 30 * PistonSpeedLimitMs / StrokeLengthMm * 1000; // multiply by 1000 to get from mm on stroke length to meters on the piston speed

        return Math.Floor(maxRpm / 100) * 100; // round down to the closest 100 IE 8769 -> 8700
    }

    // Picks a new random idle target every IdleWobbleIntervalS seconds and eases the floor toward it
    private void UpdateIdleFloor(double dt)
    {
        _timeUntilNewIdleTarget -= dt;
        if (_timeUntilNewIdleTarget <= 0)
        {
            int idleRange = (int)(TargetIdleRpm * IdleVariation);
            _idleTargetRpm = TargetIdleRpm + Random.Shared.Next(-idleRange, idleRange + 1);
            _timeUntilNewIdleTarget = IdleWobbleIntervalS;
        }
        _idleFloorRpm += (_idleTargetRpm - _idleFloorRpm) * IdleWobbleRate * dt;
    }

    // Open (+) or close (-) the throttle by the given amount; the setter keeps it between 0 and 1
    public void ThrottleControl(double amount)
    {
        ThrottlePosition += amount;
    }

    // Torque from a mean effective pressure: pressure(Pa) * displacement(m^3) / (4pi)
    // Works for BMEP (engine torque) and FMEP (friction torque)
    private double PressureToTorqueNm(double pressureBar)
    {
        double pressureInPa = pressureBar * 100000;

        double DisplacementInMCubed = DisplacementLiters / 1000;

        return pressureInPa * DisplacementInMCubed / (4 * Math.PI);
    }
}
