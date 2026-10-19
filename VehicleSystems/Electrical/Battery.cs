using EngineSim.Constants;

namespace EngineSim.Electrical;

public class Battery
{
    public bool IsBatteryConnected { get; set; } = false;
    public double BatteryChargeV = 12.6;

    public void SwitchBattery()
    {
        IsBatteryConnected = !IsBatteryConnected;
    }
}