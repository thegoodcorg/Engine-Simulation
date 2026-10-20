using EngineSim.Constants;

namespace EngineSim.VehicleSystems.Electrical;

public class Battery
{
    public double BatteryChargeV = 12.6;
    public bool IsBatteryConnected { get; private set; } = false;

    public void SwitchBattery()
    {
        IsBatteryConnected = !IsBatteryConnected;
    }
}