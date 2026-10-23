namespace EngineSim.VehicleSystems.Driveline;

// What one Drivetrain.Update decided: what the engine feels from the vehicle, and what the vehicle gets from the engine (#38)
public record DrivetrainCoupling(
    double LoadTorqueNm,        // torque holding the crank back (road load through the gears, or the slipping clutch)
    double ExtraInertiaKgM2,    // the vehicle's mass as felt at the crank. Only while locked, otherwise 0
    double DriveForceN,         // push at the tyres for VehicleBody.Update. While locked, Vehicle sets the speed instead
    ClutchState State);
