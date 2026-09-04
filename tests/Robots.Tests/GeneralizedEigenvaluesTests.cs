using System.Numerics;
using NUnit.Framework;

namespace Robots.Tests;

class GeneralizedEigenvaluesTests
{
    [Test]
    public void FindsRealComplexAndInfiniteEigenvalues()
    {
        const int order = 5;
        double[] matrixA =
        [
            2, 0, 0, 0, 0,
            0, -3, 0, 0, 0,
            0, 0, 5, 0, 0,
            0, 0, 0, 1, -2,
            0, 0, 0, 2, 1
        ];
        double[] matrixB =
        [
            1, 0, 0, 0, 0,
            0, 2, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1
        ];
        var alphaReal = new double[order];
        var alphaImaginary = new double[order];
        var beta = new double[order];

        bool success = GeneralizedEigenvalues.TryFind(
            order,
            matrixA,
            matrixB,
            alphaReal,
            alphaImaginary,
            beta,
            out int unconvergedIndex);

        Assert.That(success, Is.True);
        Assert.That(unconvergedIndex, Is.EqualTo(-1));
        AssertEigenvalues(
            alphaReal,
            alphaImaginary,
            beta,
            [new(2, 0), new(-1.5, 0), new(double.PositiveInfinity, 0), new(1, -2), new(1, 2)],
            1e-12);
    }

    [Test]
    public void FindsDenseOrder24RealPencil()
    {
        const int order = GeneralizedEigenvalues.MaximumOrder;
        var matrixA = new double[order * order];
        var matrixB = new double[order * order];
        var expected = new Complex[order];

        for (int i = 0; i < order; i++)
        {
            double alpha = i - 11.25;
            double denominator = i == 7 ? 0 : 0.75 + 0.125 * (i % 7);
            matrixA[i * order + i] = alpha;
            matrixB[i * order + i] = denominator;
            expected[i] = denominator == 0
                ? new(double.PositiveInfinity, 0)
                : new(alpha / denominator, 0);
        }

        var random = new Random(7305);

        for (int step = 0; step < order * 8; step++)
        {
            int first = random.Next(order);
            int second = random.Next(order - 1);

            if (second >= first)
                second++;

            double factor = random.NextDouble() * 0.2 - 0.1;
            AddRow(matrixA, order, first, second, factor);
            AddRow(matrixB, order, first, second, factor);
            first = random.Next(order);
            second = random.Next(order - 1);

            if (second >= first)
                second++;

            factor = random.NextDouble() * 0.2 - 0.1;
            AddColumn(matrixA, order, first, second, factor);
            AddColumn(matrixB, order, first, second, factor);
        }

        var alphaReal = new double[order];
        var alphaImaginary = new double[order];
        var beta = new double[order];

        bool success = GeneralizedEigenvalues.TryFind(
            order,
            matrixA,
            matrixB,
            alphaReal,
            alphaImaginary,
            beta,
            out int unconvergedIndex);

        Assert.That(success, Is.True, $"QZ did not converge at index {unconvergedIndex}.");
        AssertEigenvalues(alphaReal, alphaImaginary, beta, expected, 2e-9);
    }

    [Test]
    public void FindsDenseMixedSpectrum()
    {
        const int order = 6;
        double[] matrixA =
        [
            1, -2, 0, 0, 0, 0,
            2, 1, 0, 0, 0, 0,
            0, 0, -3, -0.5, 0, 0,
            0, 0, 0.5, -3, 0, 0,
            0, 0, 0, 0, 4, 0,
            0, 0, 0, 0, 0, -5
        ];
        var matrixB = new double[order * order];

        for (int i = 0; i < order; i++)
            matrixB[i * order + i] = 1;

        var random = new Random(1973);

        for (int step = 0; step < order * 8; step++)
        {
            int first = random.Next(order);
            int second = random.Next(order - 1);

            if (second >= first)
                second++;

            double factor = random.NextDouble() * 0.2 - 0.1;
            AddRow(matrixA, order, first, second, factor);
            AddRow(matrixB, order, first, second, factor);
            first = random.Next(order);
            second = random.Next(order - 1);

            if (second >= first)
                second++;

            factor = random.NextDouble() * 0.2 - 0.1;
            AddColumn(matrixA, order, first, second, factor);
            AddColumn(matrixB, order, first, second, factor);
        }

        var alphaReal = new double[order];
        var alphaImaginary = new double[order];
        var beta = new double[order];

        bool success = GeneralizedEigenvalues.TryFind(
            order,
            matrixA,
            matrixB,
            alphaReal,
            alphaImaginary,
            beta,
            out int unconvergedIndex);

        Assert.That(success, Is.True, $"QZ did not converge at index {unconvergedIndex}.");
        AssertEigenvalues(
            alphaReal,
            alphaImaginary,
            beta,
            [new(1, -2), new(1, 2), new(-3, -0.5), new(-3, 0.5), new(4, 0), new(-5, 0)],
            2e-10);
    }

    [Test]
    public void DoesNotAllocateWorkingStorage()
    {
        double[] originalA = [0, -1, 1, 0];
        double[] originalB = [1, 0, 0, 1];
        var matrixA = new double[4];
        var matrixB = new double[4];
        var alphaReal = new double[2];
        var alphaImaginary = new double[2];
        var beta = new double[2];

        originalA.CopyTo(matrixA, 0);
        originalB.CopyTo(matrixB, 0);
        _ = GeneralizedEigenvalues.TryFind(2, matrixA, matrixB, alphaReal, alphaImaginary, beta, out _);
        originalA.CopyTo(matrixA, 0);
        originalB.CopyTo(matrixB, 0);
        long before = GC.GetAllocatedBytesForCurrentThread();
        bool success = GeneralizedEigenvalues.TryFind(
            2,
            matrixA,
            matrixB,
            alphaReal,
            alphaImaginary,
            beta,
            out _);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Multiple(() =>
        {
            Assert.That(success, Is.True);
            Assert.That(allocated, Is.Zero);
        });
    }

    static void AssertEigenvalues(
        double[] alphaReal,
        double[] alphaImaginary,
        double[] beta,
        Complex[] expected,
        double tolerance)
    {
        var actual = new List<Complex>(alphaReal.Length);

        Assert.That(beta, Has.All.GreaterThanOrEqualTo(0));

        for (int i = 0; i < alphaReal.Length; i++)
        {
            actual.Add(beta[i] == 0
                ? new(double.PositiveInfinity, 0)
                : new(alphaReal[i] / beta[i], alphaImaginary[i] / beta[i]));
        }

        foreach (var value in expected)
        {
            int match = -1;
            double best = double.PositiveInfinity;

            for (int i = 0; i < actual.Count; i++)
            {
                double error = double.IsPositiveInfinity(value.Real)
                    ? double.IsPositiveInfinity(actual[i].Real) ? 0 : double.PositiveInfinity
                    : Complex.Abs(actual[i] - value);

                if (error < best)
                {
                    best = error;
                    match = i;
                }
            }

            Assert.That(match, Is.GreaterThanOrEqualTo(0), $"No match for {value}.");
            Assert.That(
                best,
                Is.LessThanOrEqualTo(tolerance * Max(1, Complex.Abs(value))),
                $"No match for {value}. Actual: {string.Join(", ", actual)}");
            actual.RemoveAt(match);
        }

        Assert.That(actual, Is.Empty);
    }

    static void AddRow(double[] matrix, int order, int target, int source, double factor)
    {
        for (int column = 0; column < order; column++)
            matrix[target * order + column] += factor * matrix[source * order + column];
    }

    static void AddColumn(double[] matrix, int order, int target, int source, double factor)
    {
        for (int row = 0; row < order; row++)
            matrix[row * order + target] += factor * matrix[row * order + source];
    }

    static double Max(double first, double second) => Math.Max(first, second);
}
