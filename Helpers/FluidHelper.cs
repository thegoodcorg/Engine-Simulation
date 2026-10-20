namespace EngineSim.Helpers;

public class FluidHelper
{
    public static double ConvertGallonsToLiters(double gal)
    {
        double liters = gal * 3.78541;
        return Math.Round(liters, 2);
    }

    public static double ConvertLitersToGallons(double liters)
    {
        double gallons = liters * 0.264172;
        return Math.Round(gallons, 2);
    }
}