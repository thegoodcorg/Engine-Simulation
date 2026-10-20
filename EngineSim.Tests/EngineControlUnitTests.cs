using EngineSim;

namespace EngineSim.Tests;

public class EngineControlUnitTests
{
    private const double Dt = 0.05;
    private const double MaxRpm = 6700;

    // A healthy running engine at idle; each test changes only what it cares about
    private static EcuInputs Inputs(
        double rpm = EngineControlUnit.TargetIdleRpm,
        bool batteryConnected = true,
        double fuelLevelPercentage = 100,
        bool engineRunning = true,
        bool cranking = false,
        double mafKgPerS = 0)
    {
        return new EcuInputs(rpm, MaxRpm, batteryConnected, fuelLevelPercentage, engineRunning, cranking, mafKgPerS);
    }

    // ---- Run checks ----

    [Fact]
    public void RunCheck_AllGood_AllowsRunWithNoMessage()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs());

        Assert.True(outputs.AllowRun);
        Assert.Null(outputs.Message);
    }

    [Fact]
    public void RunCheck_BatteryDisconnected_RefusesRun()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(batteryConnected: false));

        Assert.False(outputs.AllowRun);
        Assert.Equal("battery not connected", outputs.Message);
    }

    [Fact]
    public void RunCheck_EmptyTank_RefusesRun()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(fuelLevelPercentage: 0));

        Assert.False(outputs.AllowRun);
        Assert.Equal("fuel tank empty", outputs.Message);
    }

    [Fact]
    public void RunCheck_LowFuel_AllowsRunWithWarning()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(fuelLevelPercentage: 4));

        Assert.True(outputs.AllowRun);
        Assert.Equal("fuel level low", outputs.Message);
    }

    [Fact]
    public void RunCheck_BatteryCheckedBeforeFuel()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(batteryConnected: false, fuelLevelPercentage: 0));

        Assert.Equal("battery not connected", outputs.Message);
    }

    // ---- Spark ----

    [Fact]
    public void Spark_OnWhileRunningBelowMax()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        Assert.True(ecu.Update(Dt, Inputs()).SparkOn);
    }

    [Fact]
    public void Spark_OnWhileCranking()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(rpm: 0, engineRunning: false, cranking: true));

        Assert.True(outputs.SparkOn);
    }

    [Fact]
    public void Spark_OffWhenEngineOffAndNotCranking()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(rpm: 0, engineRunning: false));

        Assert.False(outputs.SparkOn);
    }

    [Theory]
    [InlineData(true, false)]  // running
    [InlineData(false, true)]  // cranking
    public void Spark_OffWithoutBattery(bool engineRunning, bool cranking)
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(batteryConnected: false, engineRunning: engineRunning, cranking: cranking));

        Assert.False(outputs.SparkOn);
    }

    // ---- Rev limiter ----

    [Theory]
    [InlineData(MaxRpm)]
    [InlineData(MaxRpm + 50)]
    public void RevLimiter_CutsSparkAtOrAboveMax(double rpm)
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        Assert.False(ecu.Update(Dt, Inputs(rpm: rpm)).SparkOn);
    }

    [Fact]
    public void RevLimiter_StaysCutInsideTheRestoreGap()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);
        ecu.Update(Dt, Inputs(rpm: MaxRpm)); // trip the limiter

        // Below max but still within the gap: spark stays off so it doesn't flicker
        Assert.False(ecu.Update(Dt, Inputs(rpm: MaxRpm - 1)).SparkOn);
        Assert.False(ecu.Update(Dt, Inputs(rpm: MaxRpm - 100)).SparkOn);
    }

    [Fact]
    public void RevLimiter_RestoresSparkBelowTheGap()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);
        ecu.Update(Dt, Inputs(rpm: MaxRpm)); // trip the limiter

        Assert.True(ecu.Update(Dt, Inputs(rpm: MaxRpm - 200)).SparkOn);
    }

    [Fact]
    public void RevLimiter_InsideGapWithoutTripping_KeepsSpark()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        // Climbing through the gap toward max: the limiter hasn't tripped, so spark stays on
        Assert.True(ecu.Update(Dt, Inputs(rpm: MaxRpm - 100)).SparkOn);
    }

    // ---- Idle floor ----

    [Fact]
    public void IdleFloor_StaysWithinTwoPercentOfTarget()
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        // 60 simulated seconds: plenty of new random targets
        for (int i = 0; i < 1200; i++)
        {
            double floor = ecu.Update(Dt, Inputs()).IdleFloorRpm;
            Assert.InRange(floor, EngineControlUnit.TargetIdleRpm * 0.98, EngineControlUnit.TargetIdleRpm * 1.02);
        }
    }

    // ---- Fuel (#5) ----

    [Theory]
    [InlineData(700)]
    [InlineData(3000)]
    [InlineData(6500)]
    public void Fuel_FromMafReading_IsAirPerCycleOverStoichiometricAfr(double rpm)
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);
        const double airPerCycleKg = 0.004189; // full throttle on a standard day
        double mafKgPerS = airPerCycleKg * rpm / 120; // what the MAF reads at this RPM

        EcuOutputs outputs = ecu.Update(Dt, Inputs(rpm: rpm, mafKgPerS: mafKgPerS));

        // The ECU turns the per-second reading back into per-cycle air, then divides by 14.7
        Assert.Equal(airPerCycleKg / 14.7, outputs.FuelPerCycleKg, 1e-12);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(30)] // barely turning: below the minimum RPM for fuel
    public void Fuel_NoneWhenStoppedOrBarelyTurning(double rpm)
    {
        var ecu = new EngineControlUnit(FuelProperties.PumpGas);

        EcuOutputs outputs = ecu.Update(Dt, Inputs(rpm: rpm, mafKgPerS: 0.001));

        Assert.Equal(0, outputs.FuelPerCycleKg);
    }
}
