using System.Runtime.CompilerServices;

namespace Robots;

/// <summary>
/// Recovers the 35 Raghavan-Roth coefficients for each sampled equation.
/// </summary>
/// <remarks>
/// Values and coefficients are row-major with <see cref="EquationCount"/> entries per row.
/// The input rows follow <see cref="GetSample"/>. The output rows are P-sin, P-cos,
/// P-one (nine each), then Q (eight). Input and output may be the same span.
/// </remarks>
static class RaghavanRothCoefficientRecovery
{
    public const int SampleCount = 35;
    public const int EquationCount = 14;
    public const int BufferLength = SampleCount * EquationCount;

    const int LeftGridCount = 27;
    const int RightGridCount = 9;
    const int LeftBasisCount = 9;
    const int RightBasisCount = 8;
    const double OneThird = 1.0 / 3.0;
    const double InverseSqrtThree = 0.57735026918962576450914878050196;
    const double TwoPiOverThree = 2.0943951023931953;
    const double FourPiOverThree = 4.1887902047863905;

    /// <summary>
    /// Fixed storage for either the 35x14 sampled values or recovered coefficients.
    /// </summary>
    [InlineArray(BufferLength)]
    public struct Buffer
    {
        double _element;
    }

    public static Sample GetSample(int index)
    {
        if ((uint)index >= SampleCount)
            throw new ArgumentOutOfRangeException(nameof(index));

        if (index < LeftGridCount)
        {
            int linear = index / RightGridCount;
            int remainder = index % RightGridCount;
            return new(
                Angle(linear),
                Angle(remainder / 3),
                Angle(remainder % 3),
                0,
                0);
        }

        int right = index - LeftGridCount + 1;
        return new(0, 0, 0, Angle(right / 3), Angle(right % 3));
    }

    /// <summary>
    /// Applies the inverse collocation transform without allocating.
    /// </summary>
    public static void Recover(ReadOnlySpan<double> values, Span<double> coefficients)
    {
        if (values.Length != BufferLength)
            throw new ArgumentException($"Expected {BufferLength} sampled values.", nameof(values));

        if (coefficients.Length != BufferLength)
            throw new ArgumentException($"Expected {BufferLength} coefficient values.", nameof(coefficients));

        Span<double> left = stackalloc double[LeftGridCount];
        Span<double> right = stackalloc double[RightGridCount];

        for (int equation = 0; equation < EquationCount; equation++)
        {
            for (int sample = 0; sample < LeftGridCount; sample++)
                left[sample] = values[sample * EquationCount + equation];

            right[0] = values[equation];

            for (int sample = 1; sample < RightGridCount; sample++)
                right[sample] = values[(LeftGridCount - 1 + sample) * EquationCount + equation];

            TransformLeft(left);
            TransformRight(right);

            for (int linear = 0; linear < 3; linear++)
            {
                int sourceOffset = linear * LeftBasisCount;

                for (int basis = 0; basis < LeftBasisCount; basis++)
                {
                    int coefficient = sourceOffset + basis;
                    coefficients[coefficient * EquationCount + equation] =
                        left[sourceOffset + BilinearIndex(basis)];
                }
            }

            double correction = 0;

            for (int basis = 0; basis < RightBasisCount; basis++)
            {
                double value = -right[BilinearIndex(basis)];
                coefficients[(LeftBasisCount * 3 + basis) * EquationCount + equation] = value;

                if (basis is 3 or 5 or 7)
                    correction += value;
            }

            coefficients[(LeftBasisCount * 3 - 1) * EquationCount + equation] += correction;
        }
    }

    static void TransformLeft(Span<double> values)
    {
        for (int linear = 0; linear < 3; linear++)
        {
            int offset = linear * RightGridCount;

            for (int left0 = 0; left0 < 3; left0++)
            {
                int index = offset + left0 * 3;
                Transform(values, index, index + 1, index + 2);
            }
        }

        for (int linear = 0; linear < 3; linear++)
        {
            int offset = linear * RightGridCount;

            for (int left1 = 0; left1 < 3; left1++)
                Transform(values, offset + left1, offset + left1 + 3, offset + left1 + 6);
        }

        for (int left0 = 0; left0 < 3; left0++)
        {
            for (int left1 = 0; left1 < 3; left1++)
            {
                int index = left0 * 3 + left1;
                Transform(values, index, index + RightGridCount, index + RightGridCount * 2);
            }
        }
    }

    static void TransformRight(Span<double> values)
    {
        for (int right0 = 0; right0 < 3; right0++)
        {
            int index = right0 * 3;
            Transform(values, index, index + 1, index + 2);
        }

        for (int right1 = 0; right1 < 3; right1++)
            Transform(values, right1, right1 + 3, right1 + 6);
    }

    static void Transform(Span<double> values, int first, int second, int third)
    {
        double value0 = values[first];
        double value1 = values[second];
        double value2 = values[third];
        double sum12 = value1 + value2;
        values[first] = (value1 - value2) * InverseSqrtThree;
        values[second] = (2 * value0 - sum12) * OneThird;
        values[third] = (value0 + sum12) * OneThird;
    }

    static int BilinearIndex(int index) => index switch
    {
        0 => 0,
        1 => 1,
        2 => 3,
        3 => 4,
        4 => 2,
        5 => 5,
        6 => 6,
        7 => 7,
        8 => 8,
        _ => throw new ArgumentOutOfRangeException(nameof(index))
    };

    static double Angle(int index) => index switch
    {
        0 => 0,
        1 => TwoPiOverThree,
        2 => FourPiOverThree,
        _ => throw new ArgumentOutOfRangeException(nameof(index))
    };

    public readonly record struct Sample(
        double Linear,
        double Left0,
        double Left1,
        double Right0,
        double Right1);
}
