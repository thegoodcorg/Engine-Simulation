using EngineSim.Constants;
using EngineSim.Helpers;
using EngineSim.VehicleSystems.Air;
using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #28: crank-angle cylinders breathing from the plenum instead of from outside air.
// The throttle-area tests (leak alone when shut, full bore when open) are in IntakeManifoldTests from #35.
public class ManifoldCylinderTests
{
    private const int CylinderCount = 6;
    private const double IdleRpm = 800;
    private const double IdlePlateOpening = 0.015;   // the ECU's base idle opening (#32)
    private const int SettleCycles = 20;             // the plenum settles in a couple of cycles; 20 is plenty

    private static readonly CylinderGeometry Geometry = new(95.5, 88.4, 144.2, 10.3);

    [Fact]
    public void Plenum_WithNoOutflow_FillsToAmbient()
    {
        var ambient = new AmbientConditions();
        var manifold = new IntakeManifold(ambient, EngineDisplacementLiters);

        // Pull the plenum down hard, then let nothing out and watch it refill through the shut throttle's leak
        for (int i = 0; i < 2000; i++)
            manifold.Step(0.0001, 0, 0.05);
        Assert.True(manifold.MapKpa < 50, $"pulled down to {manifold.MapKpa:F1} kPa");

        for (int i = 0; i < 100_000; i++)
            manifold.Step(0.001, 0, 0);

        Assert.Equal(ambient.AirPressureKpa, manifold.MapKpa, 0.1);
    }

    [Theory]
    [InlineData(IdleRpm, IdlePlateOpening)]
    [InlineData(2500, 0.1)]
    [InlineData(3000, 1)]
    [InlineData(6000, 1)]
    public void SteadyState_ThrottleFlowEqualsIntakeValveFlow(double rpm, double pedal)
    {
        var result = RunCoupled(rpm, pedal, new AmbientConditions());

        // Averaged over a settled cycle, what comes in through the throttle all goes out through the intake valves (within 1%)
        Assert.Equal(1, result.AverageIntakeValveFlowKgPerS / result.AverageThrottleFlowKgPerS, 0.01);
    }

    [Fact]
    public void Idle_MapIsWellBelowAmbient()
    {
        var result = RunCoupled(IdleRpm, IdlePlateOpening, new AmbientConditions());

        // ~18 kPa. A real engine idles at 30–40 kPa; this model's cylinders breathe too easily at low pressure,
        // the same gap the mean-value plenum has (~16.6 kPa)
        Assert.InRange(result.AverageMapKpa, 10, 40);
    }

    [Theory]
    [InlineData(3000)]
    [InlineData(6000)]
    public void FullThrottle_MapIsNearAmbient(double rpm)
    {
        var ambient = new AmbientConditions();
        var result = RunCoupled(rpm, 1, ambient);

        Assert.InRange(result.AverageMapKpa, ambient.AirPressureKpa - 3, ambient.AirPressureKpa); // ~100.8 at 3,000, ~99.5 at 6,000
    }

    [Fact]
    public void ThinAir_GivesLowerMap_AndLessTrappedAir()
    {
        var seaLevel = RunCoupled(3000, 1, new AmbientConditions());
        var denver = RunCoupled(3000, 1, new AmbientConditions { AirPressureKpa = 83 });

        Assert.True(denver.AverageMapKpa < seaLevel.AverageMapKpa, $"MAP {denver.AverageMapKpa:F1} vs {seaLevel.AverageMapKpa:F1} kPa");
        Assert.True(denver.TrappedMassG < seaLevel.TrappedMassG, $"trapped {denver.TrappedMassG:F3} vs {seaLevel.TrappedMassG:F3} g");
    }

    // ---- Helpers ----

    private static double EngineDisplacementLiters => CylinderCount * Geometry.SweptVolumeCc / 1000;

    private record CoupledResult(
        double AverageMapKpa,                    // plenum pressure averaged over the last cycle
        double AverageThrottleFlowKgPerS,        // air in through the throttle, averaged over the last cycle
        double AverageIntakeValveFlowKgPerS,     // all the cylinders' intake flow added up, averaged over the last cycle
        double TrappedMassG);                    // gas in cylinder 1 when its intake valve closed on the last cycle

    // Six cylinders, 120° apart, all breathing from one plenum. Every crank step:
    //   1. each cylinder sees the plenum's pressure and temperature on the far side of its intake valve
    //   2. each cylinder steps, and reports how much air it pulled through its intake valve
    //   3. the plenum steps with the cylinders' flows added up
    // Fired at 20° spark advance, run for SettleCycles cycles; the numbers come from the last one.
    private static CoupledResult RunCoupled(double rpm, double pedal, AmbientConditions ambient)
    {
        double ambientPressurePa = ambient.AirPressureKpa * 1000;

        var cylinders = new List<Cylinder>();
        for (int k = 0; k < CylinderCount; k++)
        {
            double startAngleDeg = k * 720.0 / CylinderCount;
            cylinders.Add(new Cylinder(Geometry, GasProperties.Air, startAngleDeg, ambientPressurePa, ambient.AirTempK,
                heatTransfer: new HeatTransfer(Geometry, GasProperties.Air, Engine.WallTempK),
                valvetrain: new Valvetrain(),
                fuel: FuelProperties.PumpGas)
            {
                SparkAdvanceDeg = 20,
                ExhaustPressurePa = ambientPressurePa + Engine.ExhaustBackPressureAboveAmbientPa,
            });
        }
        var manifold = new IntakeManifold(ambient, EngineDisplacementLiters);

        double stepDeg = Engine.CrankStepDeg;
        double stepSeconds = CrankHelper.StepSeconds(stepDeg, rpm);
        int stepsPerCycle = (int)Math.Round(720 / stepDeg);
        Cylinder firstCylinder = cylinders[0];
        double intakeClosesDeg = (new Valvetrain().Intake.ClosesDeg % 720 + 720) % 720;

        double mapSumKpa = 0, throttleSumKgPerS = 0, intakeSumKgPerS = 0, trappedG = 0;
        for (int cycle = 0; cycle < SettleCycles; cycle++)
        {
            bool lastCycle = cycle == SettleCycles - 1;
            for (int i = 0; i < stepsPerCycle; i++)
            {
                double firstCylinderFromDeg = firstCylinder.CrankAngleDeg % 720;

                double totalIntakeFlowKgPerS = 0;
                foreach (Cylinder cylinder in cylinders)
                {
                    cylinder.IntakePressurePa = manifold.PressurePa;
                    cylinder.IntakeTempK = ambient.AirTempK;   // the plenum stays at outside temperature
                    cylinder.Step(stepDeg, rpm);
                    totalIntakeFlowKgPerS += cylinder.IntakeFlowKgPerS;
                }
                manifold.Step(stepSeconds, pedal, totalIntakeFlowKgPerS);

                if (!lastCycle)
                    continue;

                mapSumKpa += manifold.MapKpa;
                throttleSumKgPerS += manifold.ThrottleFlowKgPerS;
                intakeSumKgPerS += totalIntakeFlowKgPerS;
                if (firstCylinderFromDeg < intakeClosesDeg && firstCylinderFromDeg + stepDeg >= intakeClosesDeg)
                    trappedG = firstCylinder.GasMassG;
            }
        }

        return new CoupledResult(mapSumKpa / stepsPerCycle, throttleSumKgPerS / stepsPerCycle, intakeSumKgPerS / stepsPerCycle, trappedG);
    }
}
