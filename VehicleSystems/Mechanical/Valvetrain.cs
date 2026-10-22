namespace EngineSim.VehicleSystems.Mechanical;

// The cams and valves that let each cylinder breathe (#27).
// Owned by Engine: one set of cams, shared by every cylinder (ticket 29 adds the per-cylinder offset).
public class Valvetrain
{
    // Placeholders from ticket 27, tuned against Nissan's curve in the dyno ticket (#30)
    private const int ValvesPerSide = 2;                // 2 intake + 2 exhaust valves per cylinder
    private const double IntakeValveDiameterMm = 37.0;
    private const double ExhaustValveDiameterMm = 31.2;
    private const double IntakeMaxLiftMm = 10.0;
    private const double ExhaustMaxLiftMm = 9.5;
    private const double DurationDeg = 240;             // both cams, measured at 0 lift
    private const double IntakeCenterlineDeg = 470;     // 110° after overlap TDC (360°)
    private const double ExhaustCenterlineDeg = 250;    // 110° before overlap TDC

    // Not in the ticket: a port slightly narrower than its valve is typical. Placeholder, tune in #30.
    private const double PortToValveDiameterRatio = 0.9;

    // Share of the geometric opening that actually flows, because real flow doesn't fill the whole gap.
    // Passed to GasFlowHelper.MassFlowKgPerS for flow through the valves.
    public const double DischargeCoefficient = 0.7;

    public CamProfile Intake { get; }
    public CamProfile Exhaust { get; }

    public Valvetrain()
    {
        Intake = new CamProfile(IntakeValveDiameterMm, IntakeValveDiameterMm * PortToValveDiameterRatio,
            ValvesPerSide, IntakeMaxLiftMm, DurationDeg, IntakeCenterlineDeg);
        Exhaust = new CamProfile(ExhaustValveDiameterMm, ExhaustValveDiameterMm * PortToValveDiameterRatio,
            ValvesPerSide, ExhaustMaxLiftMm, DurationDeg, ExhaustCenterlineDeg);
    }
}
