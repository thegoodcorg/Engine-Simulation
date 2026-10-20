namespace EngineSim;

// Facts about a fuel. Nothing decides these: they're what the fuel is.
// The ECU decides the air-fuel ratio the engine actually runs at; this only holds the stoichiometric one.
public record FuelProperties(
    double DensityKgPerL,        // how heavy a liter is
    double EnergyContentJPerKg,  // energy released by burning 1 kg
    double StoichiometricAfr)    // kg of air needed to burn 1 kg completely
{
    public static readonly FuelProperties PumpGas = new(
        DensityKgPerL: 0.745,
        EnergyContentJPerKg: 44_000_000, // 44 million joules per kilogram
        StoichiometricAfr: 14.7);        // 14.7 kg of air burns 1 kg of petrol
}
