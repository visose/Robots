using System.Runtime.CompilerServices;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

static class GeometryMath
{
    public static Point3d Circumcentre(Point3d a, Point3d b, Point3d c, Point3d d)
    {
        var ab = b - a;
        var ac = c - a;
        var ad = d - a;
        double scale = Max(ab.Length, Max(ac.Length, ad.Length));

        if (!double.IsFinite(scale) || scale == 0)
            throw new ArgumentException("Calibration points do not define a stable sphere.");

        ab /= scale;
        ac /= scale;
        ad /= scale;
        var acCrossAd = Vector3d.CrossProduct(ac, ad);
        double denominator = 2 * (ab * acCrossAd);

        if (Abs(denominator) <= 1e-10)
            throw new ArgumentException("Calibration points do not define a stable sphere.");

        var adCrossAb = Vector3d.CrossProduct(ad, ab);
        var abCrossAc = Vector3d.CrossProduct(ab, ac);
        var offset = (
            ab.SquareLength * acCrossAd
            + ac.SquareLength * adCrossAb
            + ad.SquareLength * abCrossAc) / denominator;
        return a + scale * offset;
    }

    public static double RotationAngle(Plane from, Plane to)
    {
        var rotation = to.ToTransform() * from.ToInverseTransform();
        return rotation.RotationVector().Length;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Hypot(double first, double second)
    {
        first = Abs(first);
        second = Abs(second);
        double maximum = Max(first, second);

        if (maximum == 0)
            return 0;

        first /= maximum;
        second /= maximum;
        return maximum * Sqrt(first * first + second * second);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double NormalizeAngle(double angle)
    {
        angle = IEEERemainder(angle, PI2);
        return angle <= -PI ? angle + PI2 : angle;
    }

    public static Transform RigidInverse(Transform transform)
    {
        Transform result = default;
        result.Set(
            transform.M00, transform.M10, transform.M20,
            -(transform.M00 * transform.M03 + transform.M10 * transform.M13 + transform.M20 * transform.M23),
            transform.M01, transform.M11, transform.M21,
            -(transform.M01 * transform.M03 + transform.M11 * transform.M13 + transform.M21 * transform.M23),
            transform.M02, transform.M12, transform.M22,
            -(transform.M02 * transform.M03 + transform.M12 * transform.M13 + transform.M22 * transform.M23));
        return result;
    }
}
