using EngineSim.Enums;
using EngineSim.VehicleSystems.Body;

namespace EngineSim.VehicleSystems.Driveline;

// Clutch + gearbox + final drive (#38): connects the engine's crank to the wheels.
// Each tick it works out what the engine feels from the vehicle (load and extra inertia),
// and what the vehicle gets from the engine (drive force), from the clutch state.
public class Drivetrain
{
    public const double DrivelineEfficiency = 0.90;   // 10% lost in the gears and bearings on the way to the wheels

    public Clutch Clutch { get; } = new();
    public Gearbox Gearbox { get; } = new();

    private static double TireRadiusM => VehicleBody.rollingRadiusM;

    // ---- Conversions through the current gear (only meaningful in gear) ----

    // How fast the gearbox's input (the clutch's far side) turns at this road speed (RPM)
    //   wheelRadPerS = speedMs / tireRadiusM
    //   inputRpm     = wheelRadPerS × OverallRatio × 60 / (2 × Math.PI)
    public double GearboxInputRpm(double speedMs)
    {
        double wheelRadPerS = speedMs / TireRadiusM;   // one wheel turn rolls the truck one circumference forward
        return wheelRadPerS * Gearbox.OverallRatio * 60 / (2 * Math.PI);
    }

    // The reverse: the road speed (m/s) that matches this engine RPM in the current gear. Used while locked.
    public double SpeedAtRpmMs(double engineRpm)
    {
        double engineRadPerS = engineRpm * 2 * Math.PI / 60;        // RPM → radians per second
        double wheelRadPerS = engineRadPerS / Gearbox.OverallRatio;  // the wheels turn slower than the engine
        return wheelRadPerS * TireRadiusM;                           // wheel spin → road speed
    }

    // The vehicle's mass as felt at the crank (kg·m²): massKg × tireRadiusM² / OverallRatio²
    // ~0.82 in 1st (4× the engine's own 0.19) and ~67.9 in 9th (350×): why low gears accelerate hard.
    public double ReflectedInertiaKgM2(double massKg)
    {
        return massKg * Math.Pow(TireRadiusM, 2) / Math.Pow(Gearbox.OverallRatio, 2);
    }

    // Road load as felt at the crank (Nm): roadLoadN × tireRadiusM / (OverallRatio × DrivelineEfficiency)
    public double LoadTorqueAtCrankNm(double roadLoadN)
    {
        return roadLoadN * TireRadiusM / (Gearbox.OverallRatio * DrivelineEfficiency);
    }

    // Push at the tyres from the clutch's torque (N): clutchTorqueNm × OverallRatio × DrivelineEfficiency / tireRadiusM
    // ~12,600 N at full throttle in 1st.
    public double DriveForceN(double clutchTorqueNm)
    {
        return clutchTorqueNm * Gearbox.OverallRatio * DrivelineEfficiency / TireRadiusM;
    }

    // ---- One tick ----

    // Work out the coupling between engine and vehicle for this tick.
    //   1. In neutral (or the clutch fully in): Clutch.Update(..., inNeutral: Gearbox.InNeutral) gives Disengaged.
    //      Return new(0, 0, 0, ClutchState.Disengaged): engine revs freely, vehicle coasts.
    //   2. In gear:
    //        inputRpm       = GearboxInputRpm(vehicleSpeedMs)
    //        loadTorqueNm   = LoadTorqueAtCrankNm(roadLoadN)
    //        reflectedKgM2  = ReflectedInertiaKgM2(vehicleMassKg)
    //        If locked, both sides would speed up together at:
    //          sharedAngularAccel = (engineNetTorqueNm − loadTorqueNm) / (engineInertiaKgM2 + reflectedKgM2)
    //        and the clutch would have to pass:
    //          torqueToStayLockedNm = engineNetTorqueNm − engineInertiaKgM2 × sharedAngularAccel
    //      Clutch.Update(engineRpm, inputRpm, torqueToStayLockedNm, inNeutral: false)
    //   3. Return, for the state the clutch chose:
    //        Locked:   new(loadTorqueNm, reflectedKgM2, 0, Locked)                        (Vehicle sets the speed from RPM)
    //        Slipping: new(Clutch.TorqueNm, 0, DriveForceN(Clutch.TorqueNm), Slipping)    (the clutch torque loads the engine
    //                                                                                     and drives the wheels)
    public DrivetrainCoupling Update(double dt, double engineRpm, double engineNetTorqueNm, double engineInertiaKgM2,
        double vehicleSpeedMs, double roadLoadN, double vehicleMassKg)
    {
        // 1. Neutral: nothing connects the engine to the wheels. Engine revs freely, vehicle coasts.
        //    (Return before anything uses OverallRatio: there's no ratio in neutral.)
        if (Gearbox.InNeutral)
        {
            Clutch.Update(engineRpm, gearboxInputRpm: 0, torqueToStayLockedNm: 0, torqueToMeetThisTickNm: 0, inNeutral: true);
            return new DrivetrainCoupling(0, 0, 0, Clutch.State);
        }

        // 2. In gear: what each side looks like at the crank
        double inputRpm = GearboxInputRpm(vehicleSpeedMs);
        double loadTorqueNm = LoadTorqueAtCrankNm(roadLoadN);
        double reflectedKgM2 = ReflectedInertiaKgM2(vehicleMassKg);

        // If engine and vehicle turned together, they'd speed up at this shared rate (rad/s²)...
        double sharedAngularAccel = (engineNetTorqueNm - loadTorqueNm) / (engineInertiaKgM2 + reflectedKgM2);
        // ...and the clutch would have to pass whatever the engine makes beyond what it needs to speed up its own inertia
        double torqueToStayLockedNm = engineNetTorqueNm - engineInertiaKgM2 * sharedAngularAccel;

        // While slipping, the clutch can't pass more than it takes to bring the two speeds together by the end of this tick:
        //   engine side ends at  engineSpeed + (engineNetTorque − clutchTorque) / engineInertia × dt
        //   gearbox side ends at inputSpeed  + (clutchTorque − loadTorque) / reflectedInertia × dt
        // Setting those equal and solving for clutchTorque gives:
        double speedGapRadPerS = (engineRpm - inputRpm) * 2 * Math.PI / 60;
        double torqueToMeetThisTickNm = (speedGapRadPerS / dt + engineNetTorqueNm / engineInertiaKgM2 + loadTorqueNm / reflectedKgM2)
            / (1 / engineInertiaKgM2 + 1 / reflectedKgM2);

        Clutch.Update(engineRpm, inputRpm, torqueToStayLockedNm, torqueToMeetThisTickNm, inNeutral: false);

        // 3. What each side feels, for the state the clutch chose
        return Clutch.State switch
        {
            // Pedal fully down in gear: same as neutral
            ClutchState.Disengaged => new DrivetrainCoupling(0, 0, 0, Clutch.State),

            // Turning together: the engine carries the vehicle's load and mass. Vehicle sets the speed from the RPM.
            ClutchState.Locked => new DrivetrainCoupling(loadTorqueNm, reflectedKgM2, 0, Clutch.State),

            // Slipping: the clutch's torque holds the engine back and pushes the wheels. Each side moves on its own otherwise.
            _ => new DrivetrainCoupling(Clutch.TorqueNm, 0, DriveForceN(Clutch.TorqueNm), Clutch.State),
        };
    }
}
