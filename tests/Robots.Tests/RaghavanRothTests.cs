using NUnit.Framework;

namespace Robots.Tests;

class RaghavanRothTests
{
    [Test]
    public void CoefficientRecoveryMatchesSampledEquations()
    {
        var random = new Random(603_501);
        var expected = new double[RaghavanRothCoefficientRecovery.BufferLength];

        for (int i = 0; i < expected.Length; i++)
            expected[i] = random.NextDouble() * 2 - 1;

        var samples = Sample(expected);
        var recovered = new double[expected.Length];
        RaghavanRothCoefficientRecovery.Recover(samples, recovered);
        var inPlace = (double[])samples.Clone();
        RaghavanRothCoefficientRecovery.Recover(inPlace, inPlace);

        Assert.Multiple(() =>
        {
            Assert.That(recovered, Is.EqualTo(expected).Within(3e-14));
            Assert.That(inPlace, Is.EqualTo(expected).Within(3e-14));
        });
    }

    static double[] Sample(double[] coefficients)
    {
        const int equationCount = RaghavanRothCoefficientRecovery.EquationCount;
        var values = new double[RaghavanRothCoefficientRecovery.BufferLength];
        Span<double> left = stackalloc double[9];
        Span<double> right = stackalloc double[8];

        for (int sampleIndex = 0; sampleIndex < RaghavanRothCoefficientRecovery.SampleCount; sampleIndex++)
        {
            var sample = RaghavanRothCoefficientRecovery.GetSample(sampleIndex);
            FillBasis(sample.Left0, sample.Left1, left, includeConstant: true);
            FillBasis(sample.Right0, sample.Right1, right, includeConstant: false);

            for (int equation = 0; equation < equationCount; equation++)
            {
                double value = 0;

                for (int basis = 0; basis < left.Length; basis++)
                {
                    value += (Math.Sin(sample.Linear) * coefficients[basis * equationCount + equation]
                        + Math.Cos(sample.Linear) * coefficients[(9 + basis) * equationCount + equation]
                        + coefficients[(18 + basis) * equationCount + equation]) * left[basis];
                }

                for (int basis = 0; basis < right.Length; basis++)
                    value -= coefficients[(27 + basis) * equationCount + equation] * right[basis];

                values[sampleIndex * equationCount + equation] = value;
            }
        }

        return values;
    }

    static void FillBasis(
        double first,
        double second,
        Span<double> basis,
        bool includeConstant)
    {
        double sine0 = Math.Sin(first);
        double cosine0 = Math.Cos(first);
        double sine1 = Math.Sin(second);
        double cosine1 = Math.Cos(second);
        basis[0] = sine0 * sine1;
        basis[1] = sine0 * cosine1;
        basis[2] = cosine0 * sine1;
        basis[3] = cosine0 * cosine1;
        basis[4] = sine0;
        basis[5] = cosine0;
        basis[6] = sine1;
        basis[7] = cosine1;

        if (includeConstant)
            basis[8] = 1;
    }
}
