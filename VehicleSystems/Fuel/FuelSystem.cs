
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
    // A negative amount would secretly add fuel, so it's refused.
    public void Drain(double liters)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(liters);
        FuelLevelL = Math.Max(FuelLevelL - liters, 0);
    }
}
