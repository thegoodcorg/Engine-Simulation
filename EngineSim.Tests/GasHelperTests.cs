using EngineSim.Helpers;
using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Tickets #27 and #35: compressible gas flow through a hole (valves and throttle)
public class GasHelperTests
{
    private const double Area = 0.001; // m²
    private static readonly GasProperties Air = GasProperties.Air;


    [Fact]
    public void Flow_IsZeroAtEqualPressures()
    {
        Assert.Equal(0, Flow(Area, 101_325, 288.15, 101_325, 288.15, Air));
    }

    [Fact]
    public void Flow_MatchesAHandCalculation()
    {
        // 101,325 → 90,000 Pa: pressure ratio 0.888, flow factor 0.4436
        // 0.7 × 0.001 × 101,325 / Math.Sqrt(287.05 × 288.15) × 0.4436 = 0.1094 kg/s
        Assert.Equal(0.1094, Flow(Area, 101_325, 288.15, 90_000, 288.15, Air), precision: 4);
    }

    [Fact]
    public void SwappingThePressures_ReversesTheFlow()
    {
        double forward = Flow(Area, 101_325, 288.15, 90_000, 288.15, Air);
        double backward = Flow(Area, 90_000, 288.15, 101_325, 288.15, Air);

        Assert.True(forward > 0);
        Assert.Equal(-forward, backward, precision: 12);
    }

    [Fact]
    public void ChokedFlow_StaysConstantBelowTheCriticalRatio()
    {
        // Below a ratio of 0.528 the gas in the hole is at the speed of sound: lowering the downstream pressure adds nothing
        double atHalf = Flow(Area, 200_000, 300, 100_000, 300, Air);    // ratio 0.5
        double atQuarter = Flow(Area, 200_000, 300, 50_000, 300, Air);  // ratio 0.25
        double nearVacuum = Flow(Area, 200_000, 300, 1_000, 300, Air);  // ratio 0.005

        Assert.Equal(0.3267, atHalf, precision: 4);
        Assert.Equal(atHalf, atQuarter, precision: 12);
        Assert.Equal(atHalf, nearVacuum, precision: 12);
    }

    [Fact]
    public void Flow_RisesSmoothlyIntoChoking()
    {
        // The two flow formulas meet at the critical ratio: just above and just below give the same flow
        double critical = Math.Pow(2 / 2.4, 1.4 / 0.4); // 0.528 for air
        double justAbove = Flow(Area, 200_000, 300, 200_000 * (critical + 1e-6), 300, Air);
        double justBelow = Flow(Area, 200_000, 300, 200_000 * (critical - 1e-6), 300, Air);

        Assert.Equal(justBelow, justAbove, precision: 6);
    }

    [Fact]
    public void HotterUpstreamGas_FlowsLess()
    {
        // Hot gas is thinner, so fewer kg get through the same hole: flow scales with 1 / Math.Sqrt(upstream temperature)
        double cool = Flow(Area, 101_325, 288.15, 90_000, 288.15, Air);
        double hot = Flow(Area, 101_325, 1000, 90_000, 288.15, Air);

        Assert.Equal(cool * Math.Sqrt(288.15 / 1000), hot, precision: 9);
    }

    [Fact]
    public void DischargeCoefficient_ScalesTheFlow()
    {
        // The throttle (0.8) lets more through the same hole than a valve (0.7)
        double valve = GasHelper.MassFlowKgPerS(Area, 101_325, 288.15, 90_000, 288.15, Air, 0.7);
        double throttle = GasHelper.MassFlowKgPerS(Area, 101_325, 288.15, 90_000, 288.15, Air, 0.8);

        Assert.Equal(valve * 0.8 / 0.7, throttle, precision: 12);
    }

    // Flow through a valve-sized hole, with the valves' discharge coefficient (0.7)
    private static double Flow(double areaM2, double pressureAPa, double tempAK, double pressureBPa, double tempBK, GasProperties gas) =>
        GasHelper.MassFlowKgPerS(areaM2, pressureAPa, tempAK, pressureBPa, tempBK, gas, Valvetrain.DischargeCoefficient);
}
