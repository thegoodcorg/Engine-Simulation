namespace EngineSim.VehicleSystems.Driveline;

// The gearbox and final drive (#38): which gear is in, and how many engine turns make one wheel turn.
// The real Frontier's 9-speed ratios, shifted like a manual with a clutch (no automatic or torque converter yet).
public class Gearbox
{
    public static readonly double[] GearRatios = { 5.425, 3.263, 2.250, 1.649, 1.221, 1.000, 0.862, 0.713, 0.597 }; // 1st to 9th
    public const double FinalDrive = 3.692;                                              // the differential's own reduction
    public const int TopGear = 9;

    public int Gear { get; private set; }   // 0 = neutral, 1–9 = in gear
    public bool InNeutral => Gear == 0;

    // Engine turns per wheel turn in the current gear: gearRatio × FinalDrive (e.g. 20.03 in 1st, 2.20 in 9th).
    // Only meaningful in gear. In neutral the engine and wheels aren't connected at all.
    public double OverallRatio => GearRatios[Gear - 1] * FinalDrive;

    // Change gear. Refused (returns false, gear unchanged) unless the clutch is disengaged and gear is 0 to TopGear.
    public bool TryShift(int gear, bool clutchDisengaged)
    {
        if (!clutchDisengaged || gear < 0 || gear > TopGear) return false;

        Gear = gear;
        return true;
    }
}
