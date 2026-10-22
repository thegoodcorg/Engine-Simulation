using EngineSim.Constants;
using EngineSim.VehicleSystems.Air;
using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim;

public class Engine
{
    // Tuning constants
    private const double PistonSpeedLimitMs = 20;        // max mean piston speed, sets the redline
    private const double InertiaPerLiter = 0.05;            // kg·m² per liter of displacement
    public const double CrankTimeSeconds = 0.5;
    public const double VolumetricEfficiency = 0.9;      // the cylinders only fill to 90% of their volume
    public const double ThermalEfficiency = 0.36;        // share of the fuel's energy that pushes the pistons (indicated, so friction isn't counted twice)
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


    public int CylinderCount { get; }
    public double CylinderVolumeLiters { get; }
    public double DisplacementLiters { get; }
    public double MaxRpm { get; }
    public double EngineSpeed { get; private set; }
    public double EngineTorqueNm { get; private set; } // torque being made right now
    public double NetTorque { get; private set; }
    public double Inertia { get; }
    public bool IsRunning { get; private set; }
    public double FuelBurnedG { get; private set; } // fuel used in the last Update (g), for the Vehicle to drain from the tank
    private readonly AmbientConditions _ambientConditions;
    private readonly FuelProperties _fuel;
    private readonly IntakeManifold _manifold;
    public double MafGPerS => _manifold.MafGPerS;                 // air coming in through the throttle: what a MAF sensor reads, passed to the ECU
    public double ManifoldPressureKpa => _manifold.MapKpa;        // plenum pressure (MAP), absolute

    // Air and fuel are in grams, the way an OBD2 reader shows MAF (g/s).
    // They're converted to kg only where the physics needs it (energy per kg, density in kg/L).

    // Air the cylinders really take in per cycle (g), filling from the plenum. The throttle's effect is already
    // in the plenum pressure (low at part throttle, near outside pressure when wide open), so there's no throttle factor here.
    public double AirMassPerCycleG => AirMassG(_manifold.DensityKgPerM3);


    // Torque at full throttle with a stoichiometric mix, in today's air. Changes with ambient conditions.
    // Indicated = what combustion makes, before friction. Brake = what reaches the crank, after friction
    // (the figure a manufacturer quotes as peak torque).
    // Wide open the plenum sits at outside pressure, so this uses outside air density.
    public double MaxIndicatedTorqueNm => TorqueFromFuelNm(AirMassG(_ambientConditions.AirDensityKgM3) / _fuel.StoichiometricAfr);
    // Friction depends on RPM now, so brake torque is quoted at the peak-torque speed
    public double MaxBrakeTorqueNm => MaxIndicatedTorqueNm - FrictionTorqueNm(PeakTorqueRpm);
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
        FuelProperties fuel)
    {
        // Built first: displacement and redline below read from it
        Geometry = new CylinderGeometry(boreMm, strokeMm, conRodLengthMm, compressionRatio);
        CylinderCount = cylinders;
        CylinderVolumeLiters = Geometry.SweptVolumeCc / 1000; // cc → liters
        DisplacementLiters = TotalDisplacement();
        MaxRpm = CalculateRedline();
        EngineSpeed = 0; // starts switched off
        Inertia = InertiaPerLiter * DisplacementLiters;
        _ambientConditions = ambientConditions;
        _fuel = fuel;
        _manifold = new IntakeManifold(ambientConditions, DisplacementLiters);
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

    // Advance the engine by dt seconds. fuelPerCycleG is what the ECU decided to inject.
    public void Update(double dt, bool sparkOn, double idleFloorRpm, double fuelPerCycleG)
    {
        bool combusting = IsRunning && sparkOn;
        _manifold.Update(dt, EngineSpeed, ThrottlePosition);

        // 1. Torque from the fuel that burns, minus what friction takes away.
        //    Fuel needs air to burn: in a rich mix the extra fuel goes out unburned, so the air sets a cap.
        double burnableFuelG = Math.Min(fuelPerCycleG, AirMassPerCycleG / _fuel.StoichiometricAfr);
        EngineTorqueNm = combusting ? TorqueFromFuelNm(burnableFuelG) : 0;

        // Everything injected is used up, burned or not. Whether to inject at all is the ECU's
        // decision (it cuts fuel with the spark), so the engine just uses what it's given.
        FuelBurnedG = FuelUsedG(fuelPerCycleG, EngineSpeed, dt);

        double friction = EngineSpeed > 0 ? FrictionTorqueNm(EngineSpeed) : 0;

        // Pumping loss: pulling air in against plenum vacuum and pushing exhaust out at outside pressure costs work every cycle.
        // (outside pressure − plenum pressure) × displacement (m³) ÷ (4 × Math.PI). Closed throttle at high RPM = engine braking.
        double outsidePressurePa = _ambientConditions.AirPressureKpa * 1000;
        double pumpingTorqueNm = (outsidePressurePa - _manifold.PressurePa) * (DisplacementLiters / 1000) / (4 * Math.PI);

        NetTorque = EngineTorqueNm - friction - pumpingTorqueNm;

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

    private double TotalDisplacement()
    {
        return CylinderVolumeLiters * CylinderCount;
    }

    private double CalculateRedline() // TODO: this will eventually die for a better simulation of the redline
    {
        double maxRpm = 30 * PistonSpeedLimitMs / Geometry.StrokeLengthMm * 1000; // multiply by 1000 to get from mm on stroke length to meters on the piston speed

        return Math.Floor(maxRpm / 100) * 100; // round down to the closest 100 IE 8769 -> 8700
    }

    // Fuel used over dt seconds: fuel per cycle × cycles per second × dt.
    // A 4-stroke completes one cycle every 2 revolutions, so cycles per second = RPM / 60 / 2 = RPM / 120.
    public static double FuelUsedG(double fuelPerCycleG, double rpm, double dt)
    {
        return fuelPerCycleG * rpm / 120 * dt;
    }

    // Air taken in per cycle when the cylinders fill from air of this density (kg/m³):
    // VolumetricEfficiency × density × displacement (m³), then × 1000 to turn kg into g
    private double AirMassG(double airDensityKgPerM3)
    {
        double displacementInMCubed = DisplacementLiters / 1000;

        return VolumetricEfficiency * airDensityKgPerM3 * displacementInMCubed * 1000;
    }

    // Torque from burning fuel: energy (J) × thermal efficiency = work per cycle, then ÷ (4 × Math.PI)
    // because a cycle is 2 revolutions = 4 × Math.PI radians. Energy content is per kg, so grams ÷ 1000 first.
    private double TorqueFromFuelNm(double fuelG)
    {
        double energyJ = fuelG / 1000 * _fuel.EnergyContentJPerKg;
        double workJ = energyJ * ThermalEfficiency;

        return workJ / (4 * Math.PI);
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
