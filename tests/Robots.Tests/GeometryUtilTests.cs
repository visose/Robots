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

    [TestCase(0)]
    [TestCase(1e-12)]
    [TestCase(0.004)]
    [TestCase(Math.PI - 0.004)]
    [TestCase(Math.PI)]
    [TestCase(Math.PI + 0.004)]
    public void AxisAnglePreservesRotation(double angle)
    {
        Vector3d[] axes = [Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis, new(2, -3, 4)];

        foreach (var axis in axes)
        {
            var expected = Plane.WorldXY.WithOrigin(12, -7, 3);
            _ = expected.Rotate(angle, axis, expected.Origin);
            var values = GeometryUtil.PlaneToAxisAngle(expected);
            var actual = GeometryUtil.AxisAngleToPlane(values[0], values[1], values[2], values[3], values[4], values[5]);
            AssertPlane(expected, actual);
        }
    }

    [TestCase(30, 0, 0)]
    [TestCase(0, 30, 0)]
    [TestCase(0, 0, 30)]
    [TestCase(23, -41, 67)]
    [TestCase(23, 90, 67)]
    [TestCase(23, -90, 67)]
    public void EulerXYZMatchesSuccessiveAxisRotations(double x, double y, double z)
    {
        x = x.ToRadians();
        y = y.ToRadians();
        z = z.ToRadians();
        var rotation = Transform.Rotation(x, Vector3d.XAxis, Point3d.Origin)
            * Transform.Rotation(y, Vector3d.YAxis, Point3d.Origin)
            * Transform.Rotation(z, Vector3d.ZAxis, Point3d.Origin);
        var expected = Plane.WorldXY;
        _ = expected.Transform(rotation);
        expected.Origin = new(12, -7, 3);
        var actual = GeometryUtil.EulerXYZToPlane(new(12, -7, 3, x, y, z));
        AssertPlane(expected, actual);
        var roundTrip = GeometryUtil.EulerXYZToPlane(GeometryUtil.PlaneToEulerXYZ(expected));
        AssertPlane(expected, roundTrip);
    }

    [Test]
    public void CircumcentreFindsSphereCenter()
    {
        var center = new Point3d(12, -7, 3);
        const double radius = 5;
        var actual = GeometryMath.Circumcentre(
            center + new Vector3d(radius, 0, 0),
            center + new Vector3d(0, radius, 0),
            center + new Vector3d(0, 0, radius),
            center + new Vector3d(-radius, 0, 0));

        Assert.That(actual.DistanceTo(center), Is.LessThan(1e-12));
    }

    [TestCase(0, false)]
    [TestCase(1e-12, false)]
    [TestCase(0, true)]
    public void CalibrationRejectsDegeneratePoints(double height, bool repeated)
    {
        Plane first = Plane.WorldXY.WithOrigin(1000, 0, 0);
        Plane[] planes = repeated
            ? [first, first, first, first]
            : [first, first.WithOrigin(1001, 0, 0), first.WithOrigin(1000, 1, 0), first.WithOrigin(1001, 1, height)];

        var exception = Assert.Throws<ArgumentException>(() => new Tool(Plane.WorldXY, calibrationPlanes: planes));
        Assert.That(exception!.Message, Does.Contain("do not define a stable sphere"));
    }

    [TestCase(0.001)]
    [TestCase(1)]
    [TestCase(1000)]
    public void CalibrationRecoversKnownTcp(double scale)
    {
        Point3d tcp = new(12 * scale, -7 * scale, 30 * scale);
        Point3d center = new(1000, 500, 200);
        Vector3d[] axes = [Vector3d.XAxis, Vector3d.YAxis, Vector3d.ZAxis, new(1, 2, 3)];
        var planes = axes.Select((axis, index) =>
        {
            Plane plane = Plane.WorldXY;
            _ = plane.Rotate(0.7 + index * 0.4, axis);
            plane.Origin = center - (Vector3d)plane.PointAt(tcp.X, tcp.Y, tcp.Z);
            return plane;
        }).ToArray();

        Tool tool = new(Plane.WorldXY, calibrationPlanes: planes);
        Assert.That(tool.Tcp.Origin.DistanceTo(tcp), Is.LessThan(1e-8 * Math.Max(1, scale)));
    }

    static void AssertPlane(Plane expected, Plane actual)
    {
        Assert.Multiple(() =>
        {
            Assert.That(actual.Origin.DistanceTo(expected.Origin), Is.LessThan(1e-12));
            Assert.That((actual.XAxis - expected.XAxis).Length, Is.LessThan(1e-12));
            Assert.That((actual.YAxis - expected.YAxis).Length, Is.LessThan(1e-12));
        });
    }
}
