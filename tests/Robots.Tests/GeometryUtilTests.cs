using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class GeometryUtilTests
{
    [Test]
    public void QuaternionToPlaneNormalizesInput()
    {
        var plane = GeometryUtil.QuaternionToPlane(0, 0, 0, 1, 0, 0, 1);

        Assert.Multiple(() =>
        {
            Assert.That(Vector3d.VectorAngle(plane.XAxis, Vector3d.YAxis), Is.LessThan(1e-12));
            Assert.That(Vector3d.VectorAngle(plane.YAxis, -Vector3d.XAxis), Is.LessThan(1e-12));
            Assert.That(Vector3d.VectorAngle(plane.ZAxis, Vector3d.ZAxis), Is.LessThan(1e-12));
        });
    }

    [Test]
    public void QuaternionToPlaneRejectsZeroQuaternion()
    {
        var exception = Assert.Throws<ArgumentException>(() =>
            GeometryUtil.QuaternionToPlane(0, 0, 0, 0, 0, 0, 0));

        Assert.That(exception!.Message, Does.Contain("Quaternion must not be zero."));
    }
}
