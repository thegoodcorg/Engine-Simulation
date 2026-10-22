using EngineSim.Constants;
using EngineSim.VehicleSystems.Body;

namespace EngineSim.Tests;

// Ticket #37: the truck as a mass on the road, slowed by rolling resistance, aero drag, hills and brakes
public class VehicleBodyTests
{
    private const double Dt = 0.05;
    private const double HundredKmhMs = 100 / 3.6;                                    // 27.78 m/s
    private static readonly double SeaLevelDensity = new AmbientConditions().AirDensityKgM3;  // 1.225 kg/m³

    // ---- Forces at 100 km/h ----

    [Fact]
    public void Forces_At100Kmh_MatchTheTicket()
    {
        var body = new VehicleBody(HundredKmhMs);

        // A tiny step, so the forces are read at (essentially) exactly 100 km/h
        body.Update(1e-6, driveForceN: 0, brakePedal: 0, gradePercent: 0, SeaLevelDensity);

        Assert.Equal(241, body.RollingResistanceN, 1.0);   // 0.012 × 2,050 kg × 9.81
        Assert.Equal(638, body.AeroDragN, 1.0);            // 0.5 × 1.225 × 0.45 × 3.0 m² × 27.78²
        Assert.Equal(879, body.RollingResistanceN + body.AeroDragN, 1.0);
        Assert.Equal(0, body.GradeForceN, 1e-9);
        Assert.Equal(0, body.BrakeForceN, 1e-9);
    }

    [Fact]
    public void HoldingSpeedAt100Kmh_TakesAbout24Kw()
    {
        var body = new VehicleBody(HundredKmhMs);
        body.Update(1e-6, 0, 0, 0, SeaLevelDensity);

        double powerKw = (body.RollingResistanceN + body.AeroDragN) * HundredKmhMs / 1000; // power = force × speed

        Assert.Equal(24.4, powerKw, 0.1);
    }

    [Fact]
    public void FivePercentClimb_AddsAbout1004N_AndADownhillPushesForward()
    {
        var body = new VehicleBody();

        Assert.Equal(1004, body.GradeForceAtN(Math.Atan(0.05)), 1.0);
        Assert.Equal(-1004, body.GradeForceAtN(Math.Atan(-0.05)), 1.0);
    }

    [Fact]
    public void FullBraking_IsAbout8Point8MetresPerSecondSquared()
    {
        var body = new VehicleBody(HundredKmhMs);

        body.Update(1e-6, 0, brakePedal: 1, gradePercent: 0, SeaLevelDensity);

        Assert.Equal(VehicleBody.MaxBrakeForceN, body.BrakeForceN, 1e-9);
        Assert.Equal(18000.0 / 2050, -body.AccelerationMs2 - (body.RollingResistanceN + body.AeroDragN) / VehicleBody.massKg, 0.01);
    }

    // ---- Coasting ----

    [Fact]
    public void CoastDown_From100Kmh_DeceleratesAbout0Point43()
    {
        var body = new VehicleBody(HundredKmhMs);

        body.Update(1e-6, 0, 0, 0, SeaLevelDensity);

        Assert.Equal(-0.43, body.AccelerationMs2, 0.01);
    }

    [Fact]
    public void CoastDown_SlowsFasterAtHighSpeedThanLowSpeed()
    {
        // Aero drag grows with speed², so the truck sheds speed faster on the highway than in town
        var fast = new VehicleBody(HundredKmhMs);
        var slow = new VehicleBody(30 / 3.6);

        fast.Update(Dt, 0, 0, 0, SeaLevelDensity);
        slow.Update(Dt, 0, 0, 0, SeaLevelDensity);

        Assert.True(-fast.AccelerationMs2 > -slow.AccelerationMs2,
            $"100 km/h: {fast.AccelerationMs2:F3} m/s², 30 km/h: {slow.AccelerationMs2:F3} m/s²");
    }

    [Fact]
    public void ThinAir_HasLessAeroDragAtTheSameSpeed()
    {
        // 83 kPa is roughly Denver
        double denverDensity = new AmbientConditions { AirPressureKpa = 83 }.AirDensityKgM3;
        var seaLevel = new VehicleBody(HundredKmhMs);
        var denver = new VehicleBody(HundredKmhMs);

        seaLevel.Update(1e-6, 0, 0, 0, SeaLevelDensity);
        denver.Update(1e-6, 0, 0, 0, denverDensity);

        Assert.True(denver.AeroDragN < seaLevel.AeroDragN);
        Assert.Equal(83 / 101.325, denver.AeroDragN / seaLevel.AeroDragN, 0.001); // drag scales with density
    }

    // ---- Stopping, and never rolling backwards ----

    [Fact]
    public void Brakes_StopTheTruck_AndItNeverGoesNegative()
    {
        var body = new VehicleBody(HundredKmhMs);

        for (int i = 0; i < 200; i++) // 10 s: well past the ~3.2 s it takes to stop
        {
            body.Update(Dt, 0, brakePedal: 1, gradePercent: 0, SeaLevelDensity);
            Assert.True(body.SpeedMs >= 0, $"went negative ({body.SpeedMs:F3} m/s) at {(i + 1) * Dt:F2} s");
        }

        Assert.Equal(0, body.SpeedMs);
    }

    [Fact]
    public void StoppedOnFlatGround_WithNoDriveForce_StaysStopped()
    {
        var body = new VehicleBody();

        for (int i = 0; i < 100; i++)
            body.Update(Dt, 0, brakePedal: 0, gradePercent: 0, SeaLevelDensity);

        Assert.Equal(0, body.SpeedMs);
        Assert.Equal(0, body.AccelerationMs2);
    }

    [Fact]
    public void StoppedOnADownhill_WithNoBrakes_StartsRollingForward()
    {
        var body = new VehicleBody();

        for (int i = 0; i < 20; i++) // 1 s
            body.Update(Dt, 0, brakePedal: 0, gradePercent: -5, SeaLevelDensity);

        Assert.True(body.SpeedMs > 0); // ~0.37 m/s: the hill (~1,004 N) beats rolling resistance (~241 N)
    }

    [Theory]
    [InlineData(-5)]
    [InlineData(5)]
    public void HoldingTheBrakesOnAHill_KeepsItStopped(double gradePercent)
    {
        var body = new VehicleBody();

        for (int i = 0; i < 100; i++)
            body.Update(Dt, 0, brakePedal: 1, gradePercent, SeaLevelDensity);

        Assert.Equal(0, body.SpeedMs);
    }

    [Fact]
    public void StoppedOnAnUphill_WithNoBrakes_DoesntRollBackwards()
    {
        // Reversing is out of scope: an uphill can't start a stopped truck moving
        var body = new VehicleBody();

        for (int i = 0; i < 100; i++)
            body.Update(Dt, 0, brakePedal: 0, gradePercent: 10, SeaLevelDensity);

        Assert.Equal(0, body.SpeedMs);
    }

    // ---- Through the Vehicle ----

    [Fact]
    public void Vehicle_IsFlatAndStoppedByDefault()
    {
        var vehicle = NewVehicle();

        for (int i = 0; i < 20; i++)
            vehicle.Update(Dt);

        Assert.Equal(0, vehicle.GradePercent);
        Assert.Equal(0, vehicle.SpeedMs);
    }

    [Fact]
    public void Vehicle_OnADownhill_RollsForward_AndTheBrakesHoldIt()
    {
        var rolling = NewVehicle();
        rolling.GradePercent = -5;
        var braked = NewVehicle();
        braked.GradePercent = -5;
        braked.BrakePedal = 1;

        for (int i = 0; i < 40; i++) // 2 s
        {
            rolling.Update(Dt);
            braked.Update(Dt);
        }

        Assert.True(rolling.SpeedKmh > 0);
        Assert.Equal(0, braked.SpeedKmh);
    }

    private static Vehicle NewVehicle() =>
        new(new Engine(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas));
}
