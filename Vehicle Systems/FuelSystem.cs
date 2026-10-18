using EngineSim.Constants;

namespace EngineSim;

public class FuelSystem
{
    public double TankSizeL { get; }
    public double FuelLevelL { get; set; }

    public FuelSystem(double tankSizeL)
    {
        TankSizeL = tankSizeL;
        FuelLevelL = tankSizeL; // starts full
    }

}