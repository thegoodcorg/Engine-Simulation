using EngineSim.Helpers;

namespace EngineSim.VehicleSystems.Mechanical;

// Heat leaking from the hot gas into the cylinder walls (Woschni correlation, #26).
// Woschni is empirical: fitted to measurements of real engines, so the odd powers below are just numbers to copy.
// Units are Woschni's own: pressure in kPa, bore in metres, temperature in K, gas speed in m/s.
public class HeatTransfer
{
    // Fixed parts of Woschni's formula. Not tuning knobs, so they live here next to the formula.
    private const double WoschniCoefficient = 3.26;
    private const double C1CompressionExpansion = 2.28;  // piston-motion part of the gas speed (valves shut)
    private const double C1GasExchange = 6.18;           // same, while valves are open (ticket 27)
    private const double C2Combustion = 3.24e-3;         // combustion part of the gas speed, after the spark

    private readonly CylinderGeometry _geometry;
    private readonly GasProperties _gas;

    // Snapshot of the gas when the valves shut (start of compression): Woschni's p_r, V_r, T_r
    private double _referencePressurePa;
    private double _referenceVolumeM3;
    private double _referenceTempK;

    public double WallTempK { get; set; } // settable so a cooling system can drive it later

    public HeatTransfer(CylinderGeometry geometry, GasProperties gas, double wallTempK)
    {
        _geometry = geometry;
        _gas = gas;
        WallTempK = wallTempK;
    }

    // Call once when the valves shut (for the sealed cylinder: in Cylinder's constructor)
    public void CaptureReferenceState(double pressurePa, double volumeM3, double tempK)
    {
        _referencePressurePa = pressurePa;
        _referenceVolumeM3 = volumeM3;
        _referenceTempK = tempK;
    }

    // Heat lost to the walls during one crank step (J). Positive = gas loses heat.
    // afterSpark switches on the combustion part of the gas speed (C2).
    public double HeatLossJ(double crankAngleDeg, double pressurePa, double tempK, double rpm, double stepDeg, bool afterSpark)
    {
        // dQ_loss = h × A_wall × (T − T_wall) × dt
        //   h:            how leaky the walls are right now      → CoefficientWPerM2K(...)
        //   A_wall:       metal area the gas is touching          → WallAreaM2(crankAngleDeg)
        //   T − T_wall:   how much hotter the gas is than the metal
        //   dt:           seconds this step lasts                 → StepSeconds(stepDeg, rpm)
        double gasSpeed = GasSpeedMs(crankAngleDeg, pressurePa, rpm, afterSpark);
        double h = CoefficientWPerM2K(pressurePa, tempK, gasSpeed);
        double A_wall = WallAreaM2(crankAngleDeg);
        double dt = CrankHelper.StepSeconds(stepDeg, rpm);

        return h * A_wall * (tempK - WallTempK) * dt;
    }

    // h: how easily heat leaks into the walls, per m² per kelvin of difference (W/m²·K)
    // h = 3.26 × B^−0.2 × p^0.8 × T^−0.55 × w^0.8, with B in metres and p in kPa
    private double CoefficientWPerM2K(double pressurePa, double tempK, double gasSpeedMs)
    {
        double cylDiameterM = Math.Pow(_geometry.BoreMm / 1000, -0.2); // B^-0.2
        double pressureNowKp = Math.Pow(pressurePa / 1000, 0.8);       // p^0.8
        double tempNowK = Math.Pow(tempK, -0.55);                      // T^-0.55
        double gasSpeed = Math.Pow(gasSpeedMs, 0.8);                   // w^0.8

        return WoschniCoefficient * cylDiameterM * pressureNowKp * tempNowK * gasSpeed;
    }

    // w: how fast the gas is swirling around (m/s) = piston-motion part + combustion part
    // w = C1 × Sp_mean + C2 × (Vd × T_r / (p_r × V_r)) × (p − p_motored), pressures in kPa
    private double GasSpeedMs(double crankAngleDeg, double pressurePa, double rpm, bool afterSpark)
    {
        double C1 = C1CompressionExpansion;
        double Sp_mean = MeanPistonSpeedMs(rpm);
        double C2 = C2Combustion;
        double Vd = VolumeHelper.CcToCubicMeters(_geometry.SweptVolumeCc);
        double T_r = _referenceTempK;
        double p_r = _referencePressurePa;
        double V_r = _referenceVolumeM3;
        double p = pressurePa;
        double p_motored = MotoredPressurePa(crankAngleDeg);

        // C2 = 0 before the spark: no combustion turbulence yet, only piston motion
        if (!afterSpark)
            return C1 * Sp_mean;

        // The pressure the burn added can't be negative. With heat loss, the gas can sit slightly below the
        // no-burn pressure, and a negative gas speed would make Math.Pow return NaN in the heat coefficient.
        double burnPressureRisePa = Math.Max(p - p_motored, 0);

        return C1 * Sp_mean + C2 * (Vd * T_r / (p_r * V_r) * burnPressureRisePa);

    }

    // Sp_mean: the piston's average speed (m/s) = 2 × stroke (m) × RPM / 60
    private double MeanPistonSpeedMs(double rpm)
    {
        return 2 * _geometry.StrokeLengthMm / 1000 * rpm / 60;
    }

    // p_motored: what the pressure would be right now if nothing had burned (Pa)
    // = referencePressure × Math.Pow(referenceVolume / volumeNow, heatRatio)
    private double MotoredPressurePa(double crankAngleDeg)
    {
        double volumeNowM3 = VolumeHelper.CcToCubicMeters(_geometry.Volume(crankAngleDeg));
        return _referencePressurePa * Math.Pow(_referenceVolumeM3 / volumeNowM3, _gas.HeatRatio);
    }


    // A_wall: metal the gas touches right now (m²) = head + piston crown (one piston area each)
    // + the strip of cylinder wall the piston has uncovered (Math.PI × bore × how far the piston is below TDC)
    // Distance below TDC (mm) = (Volume(crankAngleDeg) − ClearanceVolumeCc) × 1000 / PistonAreaMm2
    private double WallAreaM2(double crankAngleDeg)
    {
        // How far the piston is below TDC (mm): the swept part of the volume ÷ piston area.
        // Volume is in cc, so × 1000 for mm³ before dividing by mm².
        double sweptSoFarCc = _geometry.Volume(crankAngleDeg) - _geometry.ClearanceVolumeCc;
        double pistonDropMm = sweptSoFarCc * 1000 / _geometry.PistonAreaMm2;

        double headAndCrownMm2 = 2 * _geometry.PistonAreaMm2;
        double linerMm2 = Math.PI * _geometry.BoreMm * pistonDropMm;

        return (headAndCrownMm2 + linerMm2) / 1_000_000; // mm² → m² (1 m² = 1000 mm × 1000 mm)
    }
}
