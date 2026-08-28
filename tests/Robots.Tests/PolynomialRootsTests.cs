using NUnit.Framework;

namespace Robots.Tests;

class PolynomialRootsTests
{
    [Test]
    public void FindsRepeatedRootsAndEndpoints()
    {
        // x^2 (2x - 1)^4 (x + 1)^3 (x - 1)^2
        var polynomial = Multiply(
            Power([0, 1], 2),
            Power([-1, 2], 4),
            Power([1, 1], 3),
            Power([-1, 1], 2));

        AssertRoots(polynomial, [-1, 0, 0.5, 1]);
    }

    [Test]
    public void ExcludesRootsOutsideInterval()
    {
        // (x + 2) (x - 0.25) (x - 2)
        var polynomial = Multiply([2, 1], [-0.25, 1], [-2, 1]);

        AssertRoots(polynomial, [0.25]);
    }

    [Test]
    public void SeparatesNearRepeatedRoots()
    {
        var offset = Math.ScaleB(1, -80);
        var root = Math.ScaleB(1, -40);

        AssertRoots([-offset, 0, 1], [-root, root]);
    }

    [Test]
    public void SeparatesDeepRepresentableRoots()
    {
        double first = Math.ScaleB(1, -300);
        double second = Math.ScaleB(1, -301);
        double[] polynomial = [first * second, -(first + second), 1];
        Span<double> roots = stackalloc double[2];

        Assert.That(PolynomialRoots.TryFind(polynomial, roots, out _), Is.False);
        AssertRoots(polynomial, [second, first]);
    }

    [Test]
    public void FindsAllDegree16Roots()
    {
        var previous = new double[] { 1 };
        var current = new double[] { 0, 1 };

        for (int degree = 2; degree <= 16; degree++)
        {
            var next = Subtract(Scale(Multiply([0, 1], current), 2), previous);
            previous = current;
            current = next;
        }

        var expected = Enumerable.Range(1, 16)
            .Select(index => Math.Cos((2 * index - 1) * Math.PI / 32))
            .Order()
            .ToArray();

        AssertRoots(current, expected);
        AssertPartitionRoots(current, expected);
    }

    [Test]
    public void HandlesPowerOfTwoScaling()
    {
        double[] polynomial = [-0.25, 0, 1];
        var expected = ExactPolynomialRoots.Find(polynomial);
        var large = polynomial.Select(value => Math.ScaleB(value, 500)).ToArray();
        var small = polynomial.Select(value => Math.ScaleB(value, -500)).ToArray();

        Assert.Multiple(() =>
        {
            Assert.That(ExactPolynomialRoots.Find(large), Is.EqualTo(expected));
            Assert.That(ExactPolynomialRoots.Find(small), Is.EqualTo(expected));
        });
    }

    [Test]
    public void ResultsAreDeterministic()
    {
        var polynomial = Multiply(Power([0, 1], 2), Power([-1, 2], 4), Power([1, 1], 3));
        var expected = ExactPolynomialRoots.Find(polynomial).Select(BitConverter.DoubleToInt64Bits).ToArray();

        for (int i = 0; i < 10; i++)
        {
            var actual = ExactPolynomialRoots.Find(polynomial).Select(BitConverter.DoubleToInt64Bits).ToArray();
            Assert.That(actual, Is.EqualTo(expected));
        }
    }

    [Test]
    public void RejectsInvalidPolynomials()
    {
        double[] tooHighDegree = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];

        Assert.Multiple(() =>
        {
            Assert.That(ExactPolynomialRoots.Find([2]), Is.Empty);
            Assert.That(() => ExactPolynomialRoots.Find([0]), Throws.ArgumentException);
            Assert.That(() => ExactPolynomialRoots.Find([double.NaN, 1]), Throws.ArgumentException);
            Assert.That(() => ExactPolynomialRoots.Find(tooHighDegree), Throws.ArgumentException);
        });
    }

    static void AssertRoots(double[] polynomial, double[] expected)
    {
        var actual = ExactPolynomialRoots.Find(polynomial);

        Assert.That(actual, Has.Length.EqualTo(expected.Length));
        Assert.That(actual, Is.EqualTo(expected).Within(2e-15));
    }

    static void AssertPartitionRoots(double[] polynomial, double[] expected)
    {
        Span<double> roots = stackalloc double[16];
        bool success = PolynomialRoots.TryFind(polynomial, roots, out int count);
        var actual = roots[..count].ToArray();

        Assert.Multiple(() =>
        {
            Assert.That(success, Is.True);
            Assert.That(count, Is.EqualTo(expected.Length));
            Assert.That(actual, Is.EqualTo(expected).Within(2e-12));
        });
    }

    static double[] Power(double[] polynomial, int exponent)
    {
        double[] result = [1];

        for (int i = 0; i < exponent; i++)
            result = Multiply(result, polynomial);

        return result;
    }

    static double[] Multiply(params double[][] polynomials)
    {
        double[] result = [1];

        foreach (var polynomial in polynomials)
        {
            var product = new double[result.Length + polynomial.Length - 1];

            for (int i = 0; i < result.Length; i++)
            {
                for (int j = 0; j < polynomial.Length; j++)
                    product[i + j] += result[i] * polynomial[j];
            }

            result = product;
        }

        return result;
    }

    static double[] Scale(double[] polynomial, double factor)
    {
        return [.. polynomial.Select(value => value * factor)];
    }

    static double[] Subtract(double[] first, double[] second)
    {
        var result = new double[Math.Max(first.Length, second.Length)];

        for (int i = 0; i < result.Length; i++)
            result[i] = (i < first.Length ? first[i] : 0) - (i < second.Length ? second[i] : 0);

        return result;
    }
}
