using EngineSim.VehicleSystems.Mechanical;

namespace EngineSim.Tests;

// Ticket #23: cylinder volume, volume change per radian and lever arm vs crank angle
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
    public void Volume_RepeatsEvery360_AndIsSymmetric(double angleDeg)
    {
        Assert.Equal(_geometry.Volume(angleDeg), _geometry.Volume(angleDeg + 360), precision: 6);
        Assert.Equal(_geometry.Volume(angleDeg), _geometry.Volume(-angleDeg), precision: 6);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(90)]
    [InlineData(135)]
    [InlineData(250)]
    public void VolumeChangePerRad_MatchesFiniteDifference(double angleDeg)
    {
        // Step in degrees, divide by the step in radians: VolumeChangeCcPerRad is per radian
        const double h = 0.01;
        double finiteDifference = (_geometry.Volume(angleDeg + h) - _geometry.Volume(angleDeg - h)) / (2 * h * Math.PI / 180);
        Assert.Equal(finiteDifference, _geometry.VolumeChangeCcPerRad(angleDeg), precision: 3);
    }

    [Fact]
    public void LeverArm_IsZeroAtTdcAndBdc_AndCrankRadiusAt90()
    {
        Assert.Equal(0, _geometry.LeverArm(0), precision: 6);
        Assert.Equal(0, _geometry.LeverArm(180), precision: 6);
        Assert.Equal(_geometry.CrankRadiusMm, _geometry.LeverArm(90), precision: 6);
    }
}
