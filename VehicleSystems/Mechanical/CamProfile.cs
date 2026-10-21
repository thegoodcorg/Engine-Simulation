namespace EngineSim.VehicleSystems.Mechanical;

// One cam lobe and the valves it opens (#27): how far they lift at each crank angle, and how much gap that leaves for gas.
// Intake and exhaust use the same maths with different numbers, so Valvetrain holds two of these.
// Angles are crank degrees in the epic's 720° cycle (0° / 720° = firing TDC, 360° = overlap TDC).
public class CamProfile
{
    private const double CycleDeg = 720; // one four-stroke cycle: two crank revolutions

    public double ValveDiameterMm { get; }
    public double PortDiameterMm { get; }   // the port behind the valve: caps the flow area at high lift
    public int ValvesPerCylinder { get; }
    public double MaxLiftMm { get; }
    public double DurationDeg { get; }      // crank degrees the valve is off its seat

    // Angle of max lift. Settable so the ECU's cam phaser can slide the whole lift curve earlier or later (#31).
    public double CenterlineDeg { get; set; }

    // Where the valve starts and stops being open: half the duration either side of the centerline
    public double OpensDeg => CenterlineDeg - DurationDeg / 2;
    public double ClosesDeg => CenterlineDeg + DurationDeg / 2;

    public CamProfile(double valveDiameterMm, double portDiameterMm, int valvesPerCylinder, double maxLiftMm, double durationDeg, double centerlineDeg)
    {
        ValveDiameterMm = valveDiameterMm;
        PortDiameterMm = portDiameterMm;
        ValvesPerCylinder = valvesPerCylinder;
        MaxLiftMm = maxLiftMm;
        DurationDeg = durationDeg;
        CenterlineDeg = centerlineDeg;
    }

    // LiftMm: how far one valve is lifted off its seat at this crank angle (mm). 0 when shut.
    // lift = MaxLiftMm / 2 × (1 − Math.Cos(2 × Math.PI × degrees since opening / DurationDeg)) while open:
    // a smooth hump, 0 at both ends of the window and Lmax in the middle.
    public double LiftMm(double crankAngleDeg)
    {
        double? degreesSinceOpening = DegreesSinceOpening(crankAngleDeg);
        if (degreesSinceOpening == null)
            return 0; // shut

        // How far through the open window: 0 just opening, 0.5 at the peak, 1 just closed
        double progress = degreesSinceOpening.Value / DurationDeg;
        return MaxLiftMm / 2 * (1 - Math.Cos(2 * Math.PI * progress));
    }

    // Area gas can flow through at this crank angle (m²), for all of this cam's valves together.
    // Per valve: the ring-shaped gap (curtain area Math.PI × valve diameter × lift),
    // capped at the port's cross-section (Math.PI / 4 × port diameter²). Then × ValvesPerCylinder.
    public double FlowAreaM2(double crankAngleDeg)
    {
        double curtainMm2 = Math.PI * ValveDiameterMm * LiftMm(crankAngleDeg);   // gap around one lifted valve
        double portMm2 = Math.PI / 4 * PortDiameterMm * PortDiameterMm;     // the hole behind it
        double oneValveMm2 = Math.Min(curtainMm2, portMm2);                 // whichever is the bottleneck

        return oneValveMm2 * ValvesPerCylinder / 1_000_000;                 // mm² → m²
    }


    // How many degrees the valve has been open at crankAngleDeg, or null if it's shut.
    // crankAngleDeg keeps growing as cycles repeat (540°, 900°, 1260°...), and the open window can cross 720°/0°
    // once the cam is phased, so fold the gap since opening into 0–720° before comparing:
    //   since = ((crankAngleDeg − OpensDeg) % CycleDeg + CycleDeg) % CycleDeg   (the extra + CycleDeg keeps it positive)
    //   open while since < DurationDeg
    private double? DegreesSinceOpening(double crankAngleDeg)
    {
        double since = ((crankAngleDeg - OpensDeg) % CycleDeg + CycleDeg) % CycleDeg;
        if (since < DurationDeg)
            return since;
        else return null;
    }
}
