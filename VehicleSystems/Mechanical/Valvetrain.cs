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

    // Share of the geometric opening that actually flows, because real flow doesn't fill the whole gap
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

    // How many kg of gas go through an opening every second, between side A and side B.
    // Think of two rooms joined by a hole: gas always rushes from the higher-pressure room to the lower one.
    // Here the hole is a valve (later the throttle) and the rooms are the cylinder and a port.
    // It knows nothing about valves, just a hole size and the two sides, so ticket 28 reuses it for the throttle.
    // Positive result = A → B, negative = B → A (backflow).
    //
    // Names used below:
    //   upstreamPressurePa     pressure on the side the gas comes FROM (the higher one)
    //   upstreamTempK          temperature on that same side. Only this temperature matters:
    //                          the gas going through the hole is gas from the upstream side
    //   downstreamPressurePa   pressure on the side the gas goes TO (the lower one)
    //   direction              +1 if A is upstream (A → B), −1 if B is upstream (B → A)
    //   pressureRatio          downstreamPressurePa / upstreamPressurePa, always 0–1.
    //                          1 = equal pressures (no push), near 0 = a huge push
    //   flowFactor             how much flow that pressure ratio lets through (FlowFunction): 0 at equal pressures,
    //                          rising as the ratio drops, until it hits a ceiling (choked flow)
    //   gas.GasConstantJPerKgK 287.05 for air
    //   DischargeCoefficient   share of the hole the gas actually uses, 0.7
    //
    //   1. If the pressures are equal, nothing flows: return 0.
    //   2. Pick upstreamPressurePa, upstreamTempK, downstreamPressurePa and direction from the higher-pressure side.
    //   3. pressureRatio = downstreamPressurePa / upstreamPressurePa
    //   4. flowFactor = FlowFunction(pressureRatio, gas.HeatRatio)
    //   5. massFlowKgPerS = DischargeCoefficient × areaM2 × upstreamPressurePa
    //                       / Math.Sqrt(gas.GasConstantJPerKgK × upstreamTempK) × flowFactor
    //      In words: flow = how open the hole is × how hard the upstream gas pushes
    //      (more pressure = denser gas = more flow; hotter gas is thinner = less flow) × how much the pressure difference lets through
    //   6. return direction × massFlowKgPerS
    public static double MassFlowKgPerS(double areaM2, double pressureAPa, double tempAK, double pressureBPa, double tempBK, GasProperties gas)
    {
        if (pressureAPa == pressureBPa) return 0;

        var (upstreamPressurePa, upstreamTempK, downstreamPressurePa, direction) = pressureAPa > pressureBPa
            ? (pressureAPa, tempAK, pressureBPa, 1.0)
            : (pressureBPa, tempBK, pressureAPa, -1.0);

        double pressureRatio = downstreamPressurePa / upstreamPressurePa;

        double flowFactor = FlowFunction(pressureRatio, gas.HeatRatio);
        double massFlowKgPerS = DischargeCoefficient * areaM2 * upstreamPressurePa / Math.Sqrt(gas.GasConstantJPerKgK * upstreamTempK) * flowFactor;

        return direction * massFlowKgPerS;
    }

    // How much flow a pressure ratio (downstream ÷ upstream, 0–1) lets through. 0 when the pressures are equal.
    // heatRatio is gas.HeatRatio: the ratio of the gas's two specific heats, 1.4 for air.
    //
    // As the downstream pressure drops, more gas squeezes through, but only up to a point. Once the gas in the
    // hole is moving at the speed of sound it can't go any faster, so lowering the downstream pressure further
    // changes nothing. That's "choked" flow, and it starts at CriticalPressureRatio (0.528 for air).
    //
    //   pressureRatio > critical (not choked, a bigger difference means more flow):
    //       flowFactor = Math.Pow(pressureRatio, 1 / heatRatio)
    //                  × Math.Sqrt(2 × heatRatio / (heatRatio − 1) × (1 − Math.Pow(pressureRatio, (heatRatio − 1) / heatRatio)))
    //   pressureRatio ≤ critical (choked, flow is at its ceiling and stays there):
    //       flowFactor = Math.Sqrt(heatRatio) × Math.Pow(2 / (heatRatio + 1), (heatRatio + 1) / (2 × (heatRatio − 1)))
    //       That's a constant: 0.685 for air.
    //
    // The two formulas give the same value at the critical ratio, so there's no jump between them.
    // Checks: ratio 1 → 0, ratio 0.888 → 0.4436, any ratio ≤ 0.528 → 0.6847.
    private static double FlowFunction(double pressureRatio, double heatRatio)
    {
        double criticalPressure = CriticalPressureRatio(heatRatio);
        if (pressureRatio > criticalPressure)
        {
            return Math.Pow(pressureRatio, 1 / heatRatio) * Math.Sqrt(2 * heatRatio / (heatRatio - 1) * (1 - Math.Pow(pressureRatio, (heatRatio - 1) / heatRatio)));
        }

        return Math.Sqrt(heatRatio) * Math.Pow(2 / (heatRatio + 1), (heatRatio + 1) / (2 * (heatRatio - 1)));
    }

    // The pressure ratio where flow chokes: at or below it, the gas in the hole is already at the speed of sound.
    //   criticalRatio = Math.Pow(2 / (heatRatio + 1), heatRatio / (heatRatio − 1))
    // For air (heatRatio 1.4) that's 0.528: the flow chokes once the downstream side is below about 53% of the upstream pressure.
    private static double CriticalPressureRatio(double heatRatio)
    {
        return Math.Pow(2 / (heatRatio + 1), heatRatio / (heatRatio - 1));
    }
}
