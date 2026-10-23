using EngineSim.Constants;
using EngineSim.VehicleSystems.Driveline;

namespace EngineSim.Tests;

// Ticket #39: keyboard driving controls: clutch and brake ramps, shifting, neutral and grade
public class VehicleControlsTests
{
    private const double Dt = 0.05;

    // ---- Clutch pedal ----

    [Fact]
    public void ClutchPedalHeld_DropsEngagementToZeroStraightAway()
    {
        var vehicle = NewVehicle();
        vehicle.ClutchEngagement = 1;

        vehicle.SetClutchPedalHeld(true);
        vehicle.Update(Dt);

        Assert.Equal(0, vehicle.ClutchEngagement);
    }

    [Fact]
    public void ClutchPedalReleased_EngagesOverAbout0Point6Seconds()
    {
        var vehicle = NewVehicle();
        vehicle.SetClutchPedalHeld(true);
        vehicle.Update(Dt);

        vehicle.SetClutchPedalHeld(false);
        RunFor(vehicle, Vehicle.ClutchEngageTimeS / 2);
        Assert.Equal(0.5, vehicle.ClutchEngagement, 0.01); // half way after half the time: it ramps, it doesn't snap

        RunFor(vehicle, Vehicle.ClutchEngageTimeS / 2 + Dt);
        Assert.Equal(1, vehicle.ClutchEngagement);
    }

    [Fact]
    public void TappingTheClutch_OnlyLetsItOutPartWay()
    {
        var vehicle = NewVehicle();
        vehicle.SetClutchPedalHeld(true);
        vehicle.Update(Dt);

        vehicle.SetClutchPedalHeld(false);
        RunFor(vehicle, 0.2); // let go for a moment...
        double partWay = vehicle.ClutchEngagement;
        vehicle.SetClutchPedalHeld(true); // ...then press it again
        vehicle.Update(Dt);

        Assert.InRange(partWay, 0.2, 0.5);
        Assert.Equal(0, vehicle.ClutchEngagement);
    }

    // ---- Shifting ----

    [Fact]
    public void ShiftingWithoutTheClutch_IsRefused_WithAMessage()
    {
        var vehicle = NewVehicle();
        vehicle.SetClutchPedalHeld(false);
        RunFor(vehicle, 1); // clutch fully out

        Assert.False(vehicle.ShiftUp());
        Assert.Equal(0, vehicle.Gear);
        Assert.Equal("press clutch to shift", vehicle.DriverMessage);
    }

    [Fact]
    public void ShiftingWithTheClutchIn_ChangesGear_AndClearsTheMessage()
    {
        var vehicle = NewVehicle();
        vehicle.SetClutchPedalHeld(false);
        RunFor(vehicle, 1);
        vehicle.ShiftUp(); // refused: leaves the message up

        HoldClutch(vehicle);

        Assert.True(vehicle.ShiftUp());
        Assert.Equal(1, vehicle.Gear);
        Assert.Null(vehicle.DriverMessage);
    }

    [Fact]
    public void ShiftUp_StopsAtTopGear()
    {
        var vehicle = NewVehicle();
        HoldClutch(vehicle);

        for (int i = 0; i < Gearbox.TopGear + 3; i++) // more presses than there are gears
            vehicle.ShiftUp();

        Assert.Equal(Gearbox.TopGear, vehicle.Gear);
        Assert.False(vehicle.ShiftUp());
    }

    [Fact]
    public void ShiftDown_StopsAtFirst_NeutralIsItsOwnKey()
    {
        var vehicle = NewVehicle();
        HoldClutch(vehicle);
        vehicle.ShiftUp();
        vehicle.ShiftUp(); // 2nd

        vehicle.ShiftDown();
        Assert.Equal(1, vehicle.Gear);
        Assert.False(vehicle.ShiftDown()); // down from 1st does nothing...
        Assert.Equal(1, vehicle.Gear);

        Assert.True(vehicle.SelectNeutral()); // ...neutral has its own key
        Assert.Equal(0, vehicle.Gear);
    }

    [Fact]
    public void SelectingNeutral_NeedsTheClutchToo()
    {
        var vehicle = NewVehicle();
        HoldClutch(vehicle);
        vehicle.ShiftUp();
        vehicle.SetClutchPedalHeld(false);
        RunFor(vehicle, 1);

        Assert.False(vehicle.SelectNeutral());
        Assert.Equal(1, vehicle.Gear);
    }

    // ---- Brake ----

    [Fact]
    public void BrakeHeld_RampsUpOverAbout0Point3Seconds()
    {
        var vehicle = NewVehicle();

        vehicle.SetBrakeHeld(true);
        RunFor(vehicle, Vehicle.BrakeApplyTimeS / 2);
        Assert.Equal(0.5, vehicle.BrakePedal, 0.01);

        RunFor(vehicle, Vehicle.BrakeApplyTimeS / 2 + Dt);
        Assert.Equal(1, vehicle.BrakePedal);
    }

    [Fact]
    public void BrakeReleased_GoesBackToZero()
    {
        var vehicle = NewVehicle();
        vehicle.SetBrakeHeld(true);
        RunFor(vehicle, 1);

        vehicle.SetBrakeHeld(false);
        vehicle.Update(Dt);

        Assert.Equal(0, vehicle.BrakePedal);
    }

    // ---- Grade ----

    [Fact]
    public void Grade_StepsByOnePercent_AndClampsAtPlusOrMinus15()
    {
        var vehicle = NewVehicle();

        vehicle.ChangeGrade(+1);
        Assert.Equal(1, vehicle.GradePercent);

        for (int i = 0; i < 30; i++)
            vehicle.ChangeGrade(+1);
        Assert.Equal(Vehicle.MaxGradePercent, vehicle.GradePercent);

        for (int i = 0; i < 40; i++)
            vehicle.ChangeGrade(-1);
        Assert.Equal(-Vehicle.MaxGradePercent, vehicle.GradePercent);
    }

    // ---- Direct control still works ----

    [Fact]
    public void WithoutTheKeys_SettingTheClutchAndBrakeDirectly_IsLeftAlone()
    {
        // Tests and the dyno drive these directly; the ramps only take over once the keys are used
        var vehicle = NewVehicle();
        vehicle.ClutchEngagement = 0.4;
        vehicle.BrakePedal = 0.7;

        RunFor(vehicle, 1);

        Assert.Equal(0.4, vehicle.ClutchEngagement);
        Assert.Equal(0.7, vehicle.BrakePedal);
    }

    // ---- Helpers ----

    private static Vehicle NewVehicle() =>
        new(new Engine(95.5, 88.4, 144.2, 10.3, 6, new AmbientConditions(), FuelProperties.PumpGas));

    private static void RunFor(Vehicle vehicle, double seconds)
    {
        int ticks = (int)Math.Round(seconds / Dt);
        for (int i = 0; i < ticks; i++)
            vehicle.Update(Dt);
    }

    private static void HoldClutch(Vehicle vehicle)
    {
        vehicle.SetClutchPedalHeld(true);
        vehicle.Update(Dt);
    }
}
