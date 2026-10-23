using EngineSim.Constants;
using EngineSim.Enums;
using EngineSim.VehicleSystems.Body;
using EngineSim.VehicleSystems.Driveline;

namespace EngineSim.Tests;

// Ticket #38: clutch, gearbox and final drive connecting the engine to the truck
public class DrivetrainTests
{
    private const double Dt = 0.05;
    private const double MassKg = VehicleBody.massKg;
    private static readonly double SeaLevelDensity = new AmbientConditions().AirDensityKgM3;

    // ---- Gearing ----

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    [InlineData(8)]
    [InlineData(9)]
    public void InEachGear_RpmIsSpeedOverTireRadiusTimesOverallRatio(int gear)
    {
        var drivetrain = InGear(gear);
        double speedMs = 20;

        double expectedRpm = speedMs / VehicleBody.rollingRadiusM * Gearbox.GearRatios[gear - 1] * Gearbox.FinalDrive * 60 / (2 * Math.PI);

        Assert.Equal(expectedRpm, drivetrain.GearboxInputRpm(speedMs), 1.0);
        Assert.Equal(speedMs, drivetrain.SpeedAtRpmMs(drivetrain.GearboxInputRpm(speedMs)), 1e-9); // the two conversions undo each other
    }

    [Fact]
    public void HundredKmhInNinth_IsAbout1460Rpm()
    {
        Assert.Equal(1458, InGear(9).GearboxInputRpm(100 / 3.6), 1.0);
    }

    [Theory]
    [InlineData(1, 0.82)]
    [InlineData(9, 67.85)]
    public void ReflectedInertia_MatchesTheTable(int gear, double expectedKgM2)
    {
        // The truck's mass as felt at the crank: 2,050 kg × 0.401² / overallRatio²
        Assert.Equal(expectedKgM2, InGear(gear).ReflectedInertiaKgM2(MassKg), 0.01);
    }

    // ---- Gearbox ----

    [Fact]
    public void Shifting_IsRefusedWithTheClutchEngaged()
    {
        var gearbox = new Gearbox();

        Assert.False(gearbox.TryShift(1, clutchDisengaged: false));
        Assert.Equal(0, gearbox.Gear);
        Assert.True(gearbox.TryShift(1, clutchDisengaged: true));
        Assert.Equal(1, gearbox.Gear);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(10)]
    public void Shifting_ToAGearThatDoesntExist_IsRefused(int gear)
    {
        var gearbox = new Gearbox();

        Assert.False(gearbox.TryShift(gear, clutchDisengaged: true));
        Assert.Equal(0, gearbox.Gear);
    }

    // ---- Neutral ----

    [Fact]
    public void Neutral_ConnectsNothing()
    {
        var drivetrain = new Drivetrain();
        drivetrain.Clutch.Engagement = 1;

        DrivetrainCoupling coupling = drivetrain.Update(Dt, engineRpm: 3000, engineNetTorqueNm: 200, engineInertiaKgM2: 0.19,
            vehicleSpeedMs: 20, roadLoadN: 500, MassKg);

        Assert.Equal(new DrivetrainCoupling(0, 0, 0, ClutchState.Disengaged), coupling);
    }

    [Fact]
    public void Neutral_EngineRevsFreely_AndTheTruckCoastsAsOnItsOwn()
    {
        // In neutral at full throttle, the engine runs away while the truck just coasts, exactly as #37's body alone
        var truck = new DrivenTruck(gear: 0, speedKmh: 100, engineRpm: 1000, throttle: 1);
        var bodyAlone = new VehicleBody(100 / 3.6);

        for (int i = 0; i < 20; i++)
        {
            truck.Step();
            bodyAlone.Update(Dt, 0, 0, 0, SeaLevelDensity);
        }

        Assert.True(truck.Engine.EngineSpeed > 3000, $"engine only reached {truck.Engine.EngineSpeed:F0} RPM");
        Assert.Equal(bodyAlone.SpeedMs, truck.Body.SpeedMs, 1e-5); // the helper's one-microsecond priming step is the only difference
    }

    // ---- Clutch ----

    [Fact]
    public void Clutch_SlipsWhileTheSpeedsDiffer_AndLocksOnceTheyMatch()
    {
        var clutch = new Clutch { Engagement = 1 };

        clutch.Update(engineRpm: 2000, gearboxInputRpm: 1500, torqueToStayLockedNm: 100, torqueToMeetThisTickNm: 2000, inNeutral: false);
        Assert.Equal(ClutchState.Slipping, clutch.State);

        clutch.Update(engineRpm: 2000, gearboxInputRpm: 1995, torqueToStayLockedNm: 100, torqueToMeetThisTickNm: 120, inNeutral: false);
        Assert.Equal(ClutchState.Locked, clutch.State);
        Assert.Equal(100, clutch.TorqueNm);
    }

    [Fact]
    public void Clutch_MatchedSpeedsButMoreTorqueThanItCanGrip_Slips()
    {
        var clutch = new Clutch { Engagement = 0.5 }; // 270 Nm

        clutch.Update(engineRpm: 2000, gearboxInputRpm: 2000, torqueToStayLockedNm: 400, torqueToMeetThisTickNm: 400, inNeutral: false);

        Assert.Equal(ClutchState.Slipping, clutch.State);
        Assert.Equal(270, clutch.TorqueNm, 1e-9);
    }

    [Theory]
    [InlineData(1, 5000)]
    [InlineData(0.3, -5000)]
    [InlineData(0.6, 50)]
    [InlineData(1, -50)]
    public void ClutchTorque_NeverExceedsItsCapacity(double engagement, double torqueToMeetNm)
    {
        var clutch = new Clutch { Engagement = engagement };

        clutch.Update(engineRpm: 3000, gearboxInputRpm: 1000, torqueToStayLockedNm: torqueToMeetNm, torqueToMeetNm, inNeutral: false);

        Assert.True(Math.Abs(clutch.TorqueNm) <= clutch.CapacityNm + 1e-9, $"passed {clutch.TorqueNm:F0} Nm with {clutch.CapacityNm:F0} Nm capacity");
    }

    [Fact]
    public void Slipping_NeverOvershootsWhereTheSpeedsMeet()
    {
        // One tick of full capacity would swing the light engine ~1,350 RPM past a 280 RPM gap.
        // The clutch only passes enough to bring the two speeds together, so they meet instead of flinging past each other.
        var truck = new DrivenTruck(gear: 1, speedKmh: 2.1, engineRpm: 0, throttle: 0, running: false);
        truck.Drivetrain.Clutch.Engagement = 1;

        for (int i = 0; i < 40; i++) // 2 s
            truck.Step();

        Assert.Equal(0, truck.Body.SpeedMs, 0.01);                   // the stalled engine has dragged the truck to a stop
        Assert.Equal(ClutchState.Locked, truck.Drivetrain.Clutch.State);
    }

    // ---- Driving through the Vehicle ----

    [Fact]
    public void PullingAwayGently_InFirst_SlipsThenLocks_WithoutStalling()
    {
        var vehicle = IdlingVehicle();
        vehicle.ClutchEngagement = 0;
        Assert.True(vehicle.ShiftTo(1));
        vehicle.ThrottleControl(0.2);

        bool slipped = false;
        for (int i = 0; i <= 80; i++) // clutch out over 2 s, then hold for 2 s
        {
            vehicle.ClutchEngagement = Math.Min(1, i / 40.0);
            vehicle.Update(Dt);
            slipped |= vehicle.ClutchState == ClutchState.Slipping;
            Assert.True(vehicle.IsRunning, $"stalled at {i * Dt:F2} s");
        }

        Assert.True(slipped);
        Assert.Equal(ClutchState.Locked, vehicle.ClutchState);
        Assert.True(vehicle.SpeedKmh > 15, $"only reached {vehicle.SpeedKmh:F1} km/h");
    }

    [Fact]
    public void Locked_RpmFollowsTheRoadSpeedThroughTheGearing()
    {
        var vehicle = IdlingVehicle();
        vehicle.ClutchEngagement = 0;
        vehicle.ShiftTo(1);
        vehicle.ThrottleControl(0.2);
        for (int i = 0; i <= 80; i++)
        {
            vehicle.ClutchEngagement = Math.Min(1, i / 40.0);
            vehicle.Update(Dt);
        }

        var drivetrain = InGear(1);
        Assert.Equal(ClutchState.Locked, vehicle.ClutchState);
        Assert.Equal(drivetrain.GearboxInputRpm(vehicle.SpeedMs), vehicle.Rpm, 1.0);
    }

    [Fact]
    public void DumpingTheClutchAtIdle_InFirst_StallsTheEngine()
    {
        var vehicle = IdlingVehicle();
        vehicle.ClutchEngagement = 0;
        vehicle.ShiftTo(1);

        vehicle.ClutchEngagement = 1; // dumped
        for (int i = 0; i < 20; i++)
            vehicle.Update(Dt);

        Assert.False(vehicle.IsRunning);
        Assert.Equal(0, vehicle.SpeedKmh, 0.1);
    }

    // ---- In gear ----

    [Fact]
    public void FullThrottle_AcceleratesHarderInFirstThanThird_AtTheSameSpeed()
    {
        double firstGearAccel = AccelerationInGear(1, speedKmh: 30);
        double thirdGearAccel = AccelerationInGear(3, speedKmh: 30);

        Assert.True(firstGearAccel > thirdGearAccel, $"1st {firstGearAccel:F2} m/s², 3rd {thirdGearAccel:F2} m/s²");
    }

    [Fact]
    public void OnAFivePercentClimb_AFixedThrottleIsSlower()
    {
        double flatKmh = SettledSpeedKmh(gradePercent: 0);
        double climbKmh = SettledSpeedKmh(gradePercent: 5);

        Assert.True(climbKmh < flatKmh, $"flat {flatKmh:F1} km/h, 5% climb {climbKmh:F1} km/h");
    }

    // ---- Helpers ----

    private static Drivetrain InGear(int gear)
    {
        var drivetrain = new Drivetrain();
        drivetrain.Gearbox.TryShift(gear, clutchDisengaged: true);
        return drivetrain;
    }

    // A started vehicle, idling for 3 s
    private static Vehicle IdlingVehicle()
    {
        var vehicle = new Vehicle(NewEngine());
        vehicle.SwitchBattery();
        vehicle.SetStartKeyHeld(true);
        int crankTicks = (int)Math.Ceiling(Engine.CrankTimeSeconds / Dt) + 1;
        for (int i = 0; i < crankTicks; i++)
            vehicle.Update(Dt);
        vehicle.SetStartKeyHeld(false);
        for (int i = 0; i < 60; i++)
            vehicle.Update(Dt);
        return vehicle;
    }

    // Truck speed change over one tick at full throttle, locked in a gear at a given speed
    private static double AccelerationInGear(int gear, double speedKmh)
    {
        var truck = new DrivenTruck(gear, speedKmh, engineRpm: 0, throttle: 1, startLocked: true);
        double before = truck.Body.SpeedMs;
        truck.Step();
        return (truck.Body.SpeedMs - before) / Dt;
    }

    // Speed after 60 s at 30% throttle in 4th, starting locked at 60 km/h
    // Speed after 10 s at 30% throttle in 4th from 60 km/h. Long enough for the climb to fall behind;
    // a full settle takes a minute, which is minutes of real time on the crank-angle engine.
    private static double SettledSpeedKmh(double gradePercent)
    {
        var truck = new DrivenTruck(gear: 4, speedKmh: 60, engineRpm: 0, throttle: 0.3, startLocked: true) { GradePercent = gradePercent };
        for (int i = 0; i < 200; i++)
            truck.Step();
        return truck.Body.SpeedKmh;
    }

    private static Engine NewEngine() =>
        new(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas);

    // Engine + drivetrain + body stepped the way Vehicle.Update does, without the ECU, starter or battery,
    // so a test can start in any gear at any speed. Spark is on (at the ECU's 20°) whenever the engine is running.
    private sealed class DrivenTruck
    {
        public Engine Engine { get; } = NewEngine();
        public Drivetrain Drivetrain { get; } = new();
        public VehicleBody Body { get; }
        public double GradePercent { get; init; }

        public DrivenTruck(int gear, double speedKmh, double engineRpm, double throttle, bool running = true, bool startLocked = false)
        {
            Body = new VehicleBody(speedKmh / 3.6);
            Drivetrain.Gearbox.TryShift(gear, clutchDisengaged: true);
            Engine.ThrottlePosition = throttle;

            if (running)
                Engine.StartEngine();

            // Locked: put the engine at the speed that matches the truck through the gears, and let the plenum settle there
            double startRpm = startLocked ? Drivetrain.GearboxInputRpm(Body.SpeedMs) : engineRpm;
            Engine.HeldRpm = startRpm;
            Engine.SparkAdvanceDeg = running ? SparkAdvanceDeg : null;
            for (int i = 0; i < 20; i++)
                Engine.Update(Dt, idleThrottleOpening: 0);
            Engine.HeldRpm = null;
            if (!running)
                Engine.Stall();

            Drivetrain.Clutch.Engagement = gear == 0 ? 0 : 1;
            Body.Update(1e-6, 0, 0, GradePercent, SeaLevelDensity); // fill in the body's forces for the first road load
        }

        public void Step()
        {
            double roadLoadN = Body.RollingResistanceN + Body.AeroDragN + Body.GradeForceN + Body.BrakeForceN;
            DrivetrainCoupling coupling = Drivetrain.Update(Dt, Engine.EngineSpeed, Engine.NetTorque, Engine.Inertia,
                Body.SpeedMs, roadLoadN, MassKg);

            Engine.SparkAdvanceDeg = Engine.IsRunning ? SparkAdvanceDeg : null;
            Engine.Update(Dt, idleThrottleOpening: 0, coupling.LoadTorqueNm, coupling.ExtraInertiaKgM2);

            Body.Update(Dt, coupling.DriveForceN, brakePedal: 0, GradePercent, SeaLevelDensity);
            if (coupling.State == ClutchState.Locked)
                Body.SetSpeedMs(Drivetrain.SpeedAtRpmMs(Engine.EngineSpeed));
        }

        private const double SparkAdvanceDeg = 20;   // the ECU's spark advance
    }
}
