namespace EngineSim.VehicleSystems.Mechanical;

// The numbers from one finished open cycle (#27), returned by Cylinder.RunOpenCycle
public record CycleResult(
    double IndicatedWorkJ,          // work the gas did on the piston over the cycle
    double TrappedMassG,            // gas in the cylinder when the intake valve closed (IVC)
    double ResidualMassG,           // gas left in the cylinder when the exhaust valve closed (EVC): last cycle's exhaust, staying for this one
    double ResidualGasFraction,     // residual ÷ trapped
    double VolumetricEfficiency,    // fresh air trapped ÷ (intake air density × swept volume)
    double MassInG,                 // net gas through the intake valve over the cycle
    double MassOutG);               // net gas through the exhaust valve over the cycle. When settled, equal to MassInG
