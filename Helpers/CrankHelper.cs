namespace EngineSim.Helpers;

public class CrankHelper
{
    // Seconds the crank takes to turn stepDeg degrees at this RPM.
    // Revolutions per minute × 360° per revolution ÷ 60 s per minute = 6 × RPM degrees per second.
    public static double StepSeconds(double stepDeg, double rpm)
    {
        double degreesPerSecond = rpm * 360 / 60;
        return stepDeg / degreesPerSecond;
    }
}
