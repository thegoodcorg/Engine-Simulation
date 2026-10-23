using EngineSim.Constants;

namespace EngineSim.VehicleSystems.Body;

// The truck as a mass moving along the road (#37): slowed by rolling resistance, aero drag, hills and brakes.
// Every force is in newtons (N) at the tyres, positive = pushing backwards against the direction of travel
// (except drive force, which pushes forwards). Speed is in m/s and never goes below 0 (no reversing yet).
public class VehicleBody
{
    // all placeholders are modeling a 2024 Nissan Frontier 3.8L

    public const double massKg = 2050;
    public const double dragCoefficient = 0.45;
    public const double frontalAreaM2 = 3.0;
    public const double rollingResistance = 0.012;
    public const double rollingRadiusM = 0.401;
    public const double MaxBrakeForceN = 18000;
    public const double g = 9.81;

    public double SpeedMs { get; private set; }          // how fast the truck is going (m/s). × 3.6 for km/h
    public double SpeedKmh => SpeedMs * 3.6;

    // The forces from the last Update, for tests and the display
    public double RollingResistanceN { get; private set; }
    public double AeroDragN { get; private set; }
    public double GradeForceN { get; private set; }      // negative downhill: the hill is pushing the truck forwards
    public double BrakeForceN { get; private set; }
    public double AccelerationMs2 { get; private set; }  // positive = speeding up

    // initialSpeedMs lets tests start at a speed (e.g. 100 km/h = 27.78 m/s) without a drivetrain to get there
    public VehicleBody(double initialSpeedMs = 0)
    {
        SpeedMs = initialSpeedMs;
    }

    // While the clutch is locked (#38), the truck's speed follows the engine through the gearing,
    // so the Vehicle sets it directly instead of stepping it from forces. Never below 0.
    public void SetSpeedMs(double speedMs)
    {
        SpeedMs = Math.Max(speedMs, 0);
    }

    // Move the truck on by dt seconds.
    //   driveForceN:   push from the drivetrain at the tyres (0 in this ticket)
    //   brakePedal:    0 = off, 1 = full braking
    //   gradePercent:  hill steepness, e.g. 5 = climbing 5 m per 100 m. Negative = downhill
    //   airDensityKgPerM3: from AmbientConditions.AirDensityKgM3
    //
    // 1. Work out every force at the current speed (the methods below), and store them in the properties above
    // 2. Moving (SpeedMs > 0):
    //      netForceN       = driveForceN − rollingN − aeroN − gradeN − brakeN
    //      AccelerationMs2 = netForceN / massKg
    //      SpeedMs        += AccelerationMs2 × dt, but never below 0: rolling resistance and brakes can stop the truck,
    //                        they can't push it backwards
    // 3. Stopped (SpeedMs == 0): only the drive force or a hill can get it moving.
    //      pushN = driveForceN − gradeN        (a downhill grade makes gradeN negative, so it adds to the push)
    //      If pushN is bigger than rollingN + brakeN, it starts moving with (pushN − rollingN − brakeN) / massKg.
    //      Otherwise it stays at 0 and AccelerationMs2 is 0.
    public void Update(double dt, double driveForceN, double brakePedal, double gradePercent, double airDensityKgPerM3)

    {
        double gradeAngleRad = Math.Atan(gradePercent / 100);
        BrakeForceN = BrakeForceAtN(brakePedal);
        GradeForceN = GradeForceAtN(gradeAngleRad);
        RollingResistanceN = RollingResistanceAtN(gradeAngleRad);
        AeroDragN = AeroDragAtN(SpeedMs, airDensityKgPerM3);

        // while moving
        if (SpeedMs > 0)
        {
            double netForceN = driveForceN - RollingResistanceN - AeroDragN - GradeForceN - BrakeForceN;

            AccelerationMs2 = netForceN / massKg;
            SpeedMs = Math.Max(SpeedMs + AccelerationMs2 * dt, 0); // brakes and rolling resistance can stop the truck, never reverse it

            return;
        }

        double pushN = driveForceN - GradeForceN;
        double stopForceN = BrakeForceN + RollingResistanceN;

        AccelerationMs2 = pushN > stopForceN ? (pushN - stopForceN) / massKg : 0;
        SpeedMs += AccelerationMs2 * dt;
    }

    // Rolling resistance: the tyres flexing as they roll.
    //   gradeAngleRad = Math.Atan(gradePercent / 100)
    //   rollingN      = rollingResistance × massKg × gravity × Math.Cos(gradeAngleRad)
    // On a hill part of the weight presses along the slope instead of into the road, hence the cosine (it barely matters).
    // ~241 N on flat ground. In Update, apply it only while moving or being pushed: it opposes motion, it doesn't cause it.
    public double RollingResistanceAtN(double gradeAngleRad)
    {
        return rollingResistance * massKg * g * Math.Cos(gradeAngleRad);
    }

    // Aero drag: pushing air out of the way. Grows with the square of speed, so it dominates on the highway.
    //   aeroN = 0.5 × airDensityKgPerM3 × dragCoefficient × frontalAreaM2 × speedMs²
    // ~638 N at 100 km/h on a standard day; less in thin air.
    public double AeroDragAtN(double speedMs, double airDensityKgPerM3)
    {
        return 0.5 * airDensityKgPerM3 * dragCoefficient * frontalAreaM2 * Math.Pow(speedMs, 2);
    }

    // The hill: gravity pulling back when climbing, pushing forward when descending.
    //   gradeAngleRad = Math.Atan(gradePercent / 100)
    //   gradeN        = massKg × gravity × Math.Sin(gradeAngleRad)          (negative downhill)
    // ~1,004 N on a 5% climb.
    public double GradeForceAtN(double gradeAngleRad)
    {
        return massKg * g * Math.Sin(gradeAngleRad);
    }

    // Brakes: the pedal sets a share of the maximum braking force.
    //   brakeN = Math.Clamp(brakePedal, 0, 1) × MaxBrakeForceN
    // Full braking is ~8.8 m/s² of deceleration. Like rolling resistance, it only opposes motion.
    public double BrakeForceAtN(double brakePedal)
    {
        return Math.Clamp(brakePedal, 0, 1) * MaxBrakeForceN;
    }
}
