using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #23: cylinder volume, dV/dθ and lever arm vs crank angle
public class CylinderGeometryTests
{
    // 95.5 × 88.4 mm, rod 144.2 mm, CR 10.3 (VQ35 placeholders)
    private readonly CylinderGeometry _geometry = new(95.5, 88.4, 144.2, 10.3);

    [Fact]
    public void Volume_AtTdc_IsClearanceVolume()
    {
        Assert.Equal(_geometry.ClearanceVolumeCc, _geometry.Volume(0), precision: 6);
        Assert.Equal(68.1, _geometry.Volume(0), precision: 1);
    }

    [Fact]
    public void Volume_AtBdc_IsClearancePlusSwept()
    {
        Assert.Equal(_geometry.ClearanceVolumeCc + _geometry.SweptVolumeCc, _geometry.Volume(180), precision: 6);
        Assert.Equal(701.3, _geometry.Volume(180), precision: 1);
    }

    [Fact]
    public void Volume_At90_IsPastHalfStroke()
    {
        Assert.Equal(434.4, _geometry.Volume(90), precision: 1);
        Assert.True(_geometry.Volume(90) > _geometry.ClearanceVolumeCc + _geometry.SweptVolumeCc / 2);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(90)]
    [InlineData(135)]
    [InlineData(250)]
    public void Volume_RepeatsEvery360_AndIsSymmetric(double theta)
    {
        Assert.Equal(_geometry.Volume(theta), _geometry.Volume(theta + 360), precision: 6);
        Assert.Equal(_geometry.Volume(theta), _geometry.Volume(-theta), precision: 6);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(90)]
    [InlineData(135)]
    [InlineData(250)]
    public void DVdTheta_MatchesFiniteDifference(double theta)
    {
        // Step in degrees, divide by the step in radians: DVdTheta is per radian
        const double h = 0.01;
        double finiteDifference = (_geometry.Volume(theta + h) - _geometry.Volume(theta - h)) / (2 * h * Math.PI / 180);
        Assert.Equal(finiteDifference, _geometry.DVdTheta(theta), precision: 3);
    }

    [Fact]
    public void LeverArm_IsZeroAtTdcAndBdc_AndCrankRadiusAt90()
    {
        Assert.Equal(0, _geometry.LeverArm(0), precision: 6);
        Assert.Equal(0, _geometry.LeverArm(180), precision: 6);
        Assert.Equal(_geometry.CrankRadiusMm, _geometry.LeverArm(90), precision: 6);
    }
}
