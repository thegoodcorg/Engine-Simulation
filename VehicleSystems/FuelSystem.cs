
namespace EngineSim;

public class FuelSystem
{
    public double TankSizeL { get; }
    public double FuelLevelL { get; set; }
    public double FuelLevelPercentage => FuelLevelL / TankSizeL * 100; // worked out fresh each read
    public FuelPump Pump { get; } = new FuelPump();

    public FuelSystem(double tankSizeL)
    {
        TankSizeL = tankSizeL;
        FuelLevelL = tankSizeL; // starts full
    }
}
