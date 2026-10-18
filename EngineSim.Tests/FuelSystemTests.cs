using EngineSim;

namespace EngineSim.Tests;

public class FuelSystemTests
{
    [Fact]
    public void NewTank_StartsFull()
    {
        var fuelSystem = new FuelSystem(tankSizeL: 80);

        Assert.Equal(100, fuelSystem.FuelLevelPercentage);
    }

    [Theory]
    [InlineData(80, 80, 100)] // full
    [InlineData(80, 40, 50)]  // half
    [InlineData(80, 20, 25)]  // quarter
    [InlineData(80, 0, 0)]    // empty
    [InlineData(60, 15, 25)]  // a different tank size
    public void FuelLevelPercentage_IsLevelOverTankSize(double tankSizeL, double fuelLevelL, double expectedPercent)
    {
        var fuelSystem = new FuelSystem(tankSizeL) { FuelLevelL = fuelLevelL };

        Assert.Equal(expectedPercent, fuelSystem.FuelLevelPercentage, precision: 6);
    }

    [Fact]
    public void FuelLevelPercentage_FollowsFuelLevelChanges()
    {
        var fuelSystem = new FuelSystem(tankSizeL: 80)
        {
            FuelLevelL = 60
        };

        Assert.Equal(75, fuelSystem.FuelLevelPercentage, precision: 6);
    }
}
