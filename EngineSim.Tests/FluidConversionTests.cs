using EngineSim;
using EngineSim.Helpers;

namespace EngineSim.Tests;

public class FluidConversionTests
{

    [Theory]
    [InlineData(5, 18.93)]
    [InlineData(23, 87.06)]
    [InlineData(40, 151.42)]

    public void ConvertingGallonsToLitersIsAccurate(double gallons, double expectedLiters)
    {
        var result = FluidHelper.ConvertGallonsToLiters(gallons);

        Assert.Equal(result, expectedLiters);
    }

    [Theory]
    [InlineData(10, 2.64)]
    [InlineData(50, 13.21)]
    [InlineData(75, 19.81)]



    public void ConvertingLitersToGallonsIsAccurate(double liters, double expectedGallons)
    {
        var result = FluidHelper.ConvertLitersToGallons(liters);

        Assert.Equal(result, expectedGallons);
    }


}
