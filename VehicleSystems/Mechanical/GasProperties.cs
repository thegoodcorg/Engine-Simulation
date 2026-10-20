namespace EngineSim.VehicleSystems.Mechanical;

// Facts about the gas trapped in a cylinder. Nothing decides these: they're what the gas is.
// γ is held constant for now, even though real air's γ drops a little as it heats up.
public record GasProperties(
    double GasConstantJPerKgK, // R: links pressure, volume and temperature in p = m·R·T / V
    double Gamma)              // γ = cp / cv, the ratio of specific heats
{
    // cv: energy to warm 1 kg of gas by 1 K at constant volume. Worked out from R and γ so the three always agree.
    public double CvJPerKgK => GasConstantJPerKgK / (Gamma - 1);

    public static readonly GasProperties Air = new(
        GasConstantJPerKgK: 287.05, // same value AmbientConditions uses for air density
        Gamma: 1.4);                // cv = 287.05 / 0.4 = 717.6 J/(kg·K)
}
