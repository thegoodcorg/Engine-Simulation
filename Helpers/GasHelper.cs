using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Helpers;

// Compressible gas flow through a hole: valves, the throttle, anything with a pressure on each side.
public class GasHelper
{
    // How many kg of gas go through an opening every second, between side A and side B.
    // Think of two rooms joined by a hole: gas always rushes from the higher-pressure room to the lower one.
    // The hole is a valve (the rooms are the cylinder and a port) or the throttle (the outside air and the plenum).
    // It knows nothing about valves or throttles, just a hole size and the two sides: the valves (#27) and the throttle (#35) both use it.
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
    //   dischargeCoefficient   share of the hole the gas actually uses: 0.7 for the valves (Valvetrain.DischargeCoefficient), 0.8 for the throttle
    //
    //   1. If the pressures are equal, nothing flows: return 0.
    //   2. Pick upstreamPressurePa, upstreamTempK, downstreamPressurePa and direction from the higher-pressure side.
    //   3. pressureRatio = downstreamPressurePa / upstreamPressurePa
    //   4. flowFactor = FlowFunction(pressureRatio, gas.HeatRatio)
    //   5. massFlowKgPerS = dischargeCoefficient × areaM2 × upstreamPressurePa
    //                       / Math.Sqrt(gas.GasConstantJPerKgK × upstreamTempK) × flowFactor
    //      In words: flow = how open the hole is × how hard the upstream gas pushes
    //      (more pressure = denser gas = more flow; hotter gas is thinner = less flow) × how much the pressure difference lets through
    //   6. return direction × massFlowKgPerS
    public static double MassFlowKgPerS(double areaM2, double pressureAPa, double tempAK, double pressureBPa, double tempBK, GasProperties gas,
        double dischargeCoefficient)
    {
        if (pressureAPa == pressureBPa) return 0;

        var (upstreamPressurePa, upstreamTempK, downstreamPressurePa, direction) = pressureAPa > pressureBPa
            ? (pressureAPa, tempAK, pressureBPa, 1.0)
            : (pressureBPa, tempBK, pressureAPa, -1.0);

        double pressureRatio = downstreamPressurePa / upstreamPressurePa;

        double flowFactor = FlowFunction(pressureRatio, gas.HeatRatio);
        double massFlowKgPerS = dischargeCoefficient * areaM2 * upstreamPressurePa / Math.Sqrt(gas.GasConstantJPerKgK * upstreamTempK) * flowFactor;

        return direction * massFlowKgPerS;
    }

    public static double ConvertKpaToPsi(double kpa)
    {
        return kpa * 0.145038; // 1 kPa = 0.145038 psi
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
