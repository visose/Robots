using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

class WristPolynomialTests
{
    static readonly double[] HalfAngles =
        [1.0 / 5, 1.0 / 3, -1.0 / 4, 2.0 / 5, -1.0 / 3, 1.0 / 2];

    [Test]
    public void PoWaContainsJoint6()
    {
        double[] a = [0.180, 0.920, 0, 0, 0.080, 0];
        double[] d = [0.3825, 0, -0.0601, 0.856, 0.138, 0.120];

        AssertPolynomial(a, d);
    }

    [Test]
    public void GriffithGoFaContainsJoint6()
    {
        double[] a = [0, 0.707, 0.110, 0, 0.080, 0];
        double[] d = [0.338, 0, 0, 0.534, 0, 0.101];

        AssertPolynomial(a, d);
    }

    [TestCase(0)]
    [TestCase(Math.PI / 2)]
    [TestCase(Math.PI)]
    [TestCase(-Math.PI / 2)]
    public void BuildsEachHalfAngleChart(double phase)
    {
        double[] a = [0, 0.707, 0.110, 0, 0.080, 0];
        double[] d = [0.338, 0, 0, 0.534, 0, 0.101];
        double[] joints = [.. HalfAngles.Select(value => 2 * Math.Atan(value))];
        joints[5] = phase;
        var target = Forward(a, d, joints);
        Span<double> coefficients = stackalloc double[WristPolynomial.MaxDegree + 1];

        bool success = WristPolynomial.TryBuild(
            a,
            d,
            target,
            phase,
            coefficients,
            out var result);
        double residual = Evaluate(coefficients[..(result.Degree + 1)], 0);

        Assert.Multiple(() =>
        {
            Assert.That(success, Is.True);
            Assert.That(result.Degree, Is.LessThanOrEqualTo(16));
            Assert.That(Math.Abs(residual), Is.LessThan(1e-8));
        });
    }

    static void AssertPolynomial(double[] a, double[] d)
    {
        double[] joints = [.. HalfAngles.Select(value => 2 * Math.Atan(value))];
        var target = Forward(a, d, joints);
        Span<double> coefficients = stackalloc double[WristPolynomial.MaxDegree + 1];

        bool success = WristPolynomial.TryBuild(
            a,
            d,
            target,
            phase: 0,
            coefficients,
            out var result);

        Assert.That(success, Is.True);

        double residual = Evaluate(
            coefficients[..(result.Degree + 1)],
            HalfAngles[5]);

        Assert.Multiple(() =>
        {
            Assert.That(result.Degree, Is.EqualTo(16));
            Assert.That(result.IsStable, Is.True);
            Assert.That(result.RelativeRemainder, Is.LessThanOrEqualTo(1e-9));
            Assert.That(Math.Abs(residual), Is.LessThan(1e-8));
        });
    }

    static Transform Forward(double[] a, double[] d, double[] joints)
    {
        double[] cosAlpha = [0, 1, 0, 0, 0, 1];
        double[] sinAlpha = [1, 0, 1, -1, 1, 0];
        var transform = Transform.Identity;

        for (int i = 0; i < 6; i++)
        {
            double cos = Math.Cos(joints[i]);
            double sin = Math.Sin(joints[i]);
            Transform joint = default;
            joint.Set(
                cos, -sin * cosAlpha[i], sin * sinAlpha[i], a[i] * cos,
                sin, cos * cosAlpha[i], -cos * sinAlpha[i], a[i] * sin,
                0, sinAlpha[i], cosAlpha[i], d[i]);
            transform *= joint;
        }

        return transform;
    }

    static double Evaluate(ReadOnlySpan<double> coefficients, double value)
    {
        double result = 0;

        for (int i = coefficients.Length - 1; i >= 0; i--)
            result = result * value + coefficients[i];

        return result;
    }
}
