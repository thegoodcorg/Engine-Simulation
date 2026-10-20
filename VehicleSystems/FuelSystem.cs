
namespace EngineSim;

public class FuelSystem
{
    public double TankSizeL { get; }
    public double FuelLevelL { get; set; }
    public double FuelLevelPercentage => FuelLevelL / TankSizeL * 100; // worked out fresh each read
    public FuelProperties Fuel { get; } = FuelProperties.PumpGas;      // what's in the tank
    public FuelPump Pump { get; } = new FuelPump();

    public FuelSystem(double tankSizeL)
    {
        TankSizeL = tankSizeL;
        FuelLevelL = tankSizeL; // starts full
    }

    // Take fuel out of the tank. The level never goes below empty.
    public void Drain(double liters)
    {
        FuelLevelL = Math.Max(FuelLevelL - liters, 0);
    }
}
