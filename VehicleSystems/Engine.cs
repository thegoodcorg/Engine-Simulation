using EngineSim.Constants;

namespace EngineSim;

public class Engine
{
    // Tuning constants
    private const double PistonSpeedLimitMs = 20;        // max mean piston speed, sets the redline
    private const double MinThrottle = 0.06;             // throttle never fully closes, like an idle air bypass
    private const double BmepBar = 12.5;                 // brake mean effective pressure, used for torque
    private const double FmepBar = 1.2;                    // Friction mean effective pressure, used for torque
    private const double InertiaPerLiter = 0.05;            // kg·m² per liter of displacement
    public const double CrankTimeSeconds = 0.5;

    public double BoreMm { get; }  // Diameter of the cylinder
    public double StrokeLengthMm { get; } // distance the piston moves up/down
    public int CylinderCount { get; }
    public double CylinderVolumeLiters { get; }
    public double DisplacementLiters { get; }
    public double MaxRpm { get; }
    public double TargetIdleRpm { get; } = 700;
    public double EngineSpeed { get; private set; }
    public double MaxTorqueNm { get; } // BMEP(Pa) * dsiplacement(m^3)/(4pi)
    public double FrictionTorque { get; } // FMEP * displacement / (4pi)
    public double EngineTorqueNm { get; private set; } // torque being made right now
    public double NetTorque { get; private set; }
    public double Inertia { get; }
    public bool IsRunning { get; private set; }

    // MinThrottle = released, 1.0 = wide open. Values outside that range get clamped.
    private double _throttlePosition = MinThrottle;
    public double ThrottlePosition
    {
        get => _throttlePosition;
        set => _throttlePosition = Math.Clamp(value, MinThrottle, 1.0);
    }

    public Engine(double bore, double stroke, int cylinders, AmbientConditions ambientConditions)
    {
        BoreMm = bore;
        StrokeLengthMm = stroke;
        CylinderCount = cylinders;
        CylinderVolumeLiters = CalculateCylinderVolumeLiters();
        DisplacementLiters = TotalDisplacement();
        MaxRpm = CalculateRedline();
        EngineSpeed = 0; // starts switched off
        MaxTorqueNm = PressureToTorqueNm(BmepBar);
        FrictionTorque = PressureToTorqueNm(FmepBar);
        Inertia = InertiaPerLiter * DisplacementLiters;
    }
    public void StartEngine()
    {
        IsRunning = true;
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

    // Advance the engine by dt seconds
    public void Update(double dt, bool sparkOn, double idleFloorRpm)
    {
        bool combusting = IsRunning && sparkOn;
        // 1. Torque at the current throttle, minus what friction takes away
        EngineTorqueNm = combusting ? MaxTorqueNm * ThrottlePosition : 0;

        double friction = EngineSpeed > 0 ? FrictionTorque : 0;

        NetTorque = EngineTorqueNm - friction;

        // 2. Rotational version of F = ma: angular acceleration = torque / inertia (rad/s²)
        double angularAcceleration = NetTorque / Inertia;

        // 3. Convert rad/s² to RPM per second, then scale by the time step
        EngineSpeed += angularAcceleration * 60 / (2 * Math.PI) * dt;


        // 4. While running, the ECU's idle floor holds RPM up. When off, it spins down to 0.
        //    MaxRpm stays as a hard ceiling, because one 0.05 s tick at full throttle can overshoot
        //    the rev limiter by several hundred RPM before it gets a chance to cut spark.
        double floor = IsRunning ? idleFloorRpm : 0;
        EngineSpeed = Math.Clamp(EngineSpeed, floor, MaxRpm);
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

    // Torque from a mean effective pressure: pressure(Pa) * displacement(m^3) / (4pi)
    // Works for BMEP (engine torque) and FMEP (friction torque)
    private double PressureToTorqueNm(double pressureBar)
    {
        double pressureInPa = pressureBar * 100000;

        double displacementInMCubed = DisplacementLiters / 1000;

        return pressureInPa * displacementInMCubed / (4 * Math.PI);
    }
}
