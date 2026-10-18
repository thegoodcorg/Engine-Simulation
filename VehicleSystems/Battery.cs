using EngineSim.Constants;

namespace EngineSim;

public class Battery
{
    public bool IsBatteryConnected { get; set; } = false;
    public double BatteryChargeV = 12.6;

    public void SwitchBattery()
    {
        IsBatteryConnected = !IsBatteryConnected;
    }
}