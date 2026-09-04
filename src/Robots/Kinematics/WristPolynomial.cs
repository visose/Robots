using System.Runtime.CompilerServices;
using Rhino.Geometry;

namespace Robots;

/// <summary>
/// Builds the degree-16 pose eliminant for the supported offset-wrist DH class.
/// All lengths and the target translation use the caller's scale.
/// </summary>
static class WristPolynomial
{
    public const int MaxDegree = 16;

    const int PolynomialCapacity = 41;
    const int MaxRadialDegree = 6;
    const int MaxRadialTangentDegree = 20;
    const int RadialStride = MaxRadialTangentDegree + 1;
    const int RadialCapacity = (MaxRadialDegree + 1) * RadialStride;
    const double ReliableRemainder = 1e-9;
    const double MaxRemainder = 1e-7;
    const double DegreeTolerance = 1e-13;

    public readonly record struct Result(
        int Degree,
        double RelativeRemainder)
    {
        public bool IsStable => RelativeRemainder <= ReliableRemainder;
    }

    /// <summary>
    /// Returns coefficients in ascending powers of
    /// <c>t = tan((joint6 - phase) / 2)</c>.
    /// </summary>
    public static bool TryBuild(
        ReadOnlySpan<double> a,
        ReadOnlySpan<double> d,
        Transform target,
        double phase,
        Span<double> coefficients,
        out Result result)
    {
        if (a.Length != 6)
            throw new ArgumentException("Six DH A parameters are required.", nameof(a));

        if (d.Length != 6)
            throw new ArgumentException("Six DH D parameters are required.", nameof(d));

        if (coefficients.Length < MaxDegree + 1)
            throw new ArgumentException("The coefficient buffer is too small.", nameof(coefficients));

        if (!AreFinite(a) || !AreFinite(d) || !double.IsFinite(phase))
            throw new ArgumentException("DH parameters and chart phase must be finite.");

        if (!IsFinite(target))
            throw new ArgumentException("Target transform must be finite.", nameof(target));

        result = default;

        Polynomial.Create([1, 0, 1], out var delta);
        Polynomial.Create([1, 0, -1], out var tangentCos);
        Polynomial.Create([0, 2], out var tangentSin);
        double phaseCos = SnapTrig(Math.Cos(phase));
        double phaseSin = SnapTrig(Math.Sin(phase));
        Polynomial.Combine(
            in tangentCos,
            phaseCos,
            in tangentSin,
            -phaseSin,
            out var cosNumerator);
        Polynomial.Combine(
            in tangentCos,
            phaseSin,
            in tangentSin,
            phaseCos,
            out var sinNumerator);

        BuildComponent(
            target.M00,
            target.M01,
            target.M02,
            target.M03,
            a[4],
            d[4],
            d[5],
            in delta,
            in cosNumerator,
            in sinNumerator,
            out var y5X,
            out var x);
        BuildComponent(
            target.M10,
            target.M11,
            target.M12,
            target.M13,
            a[4],
            d[4],
            d[5],
            in delta,
            in cosNumerator,
            in sinNumerator,
            out var y5Y,
            out var y);
        BuildComponent(
            target.M20,
            target.M21,
            target.M22,
            target.M23,
            a[4],
            d[4],
            d[5],
            in delta,
            in cosNumerator,
            in sinNumerator,
            out var vertical,
            out var z);

        Polynomial.Square(in delta, out var deltaSquared);
        Polynomial.Square(in x, out var tempPolynomial);
        Polynomial.Square(in y, out var planarRadius);
        Polynomial.AddScaled(ref planarRadius, in tempPolynomial, 1);
        Polynomial.Scale(in deltaSquared, d[2] * d[2], out tempPolynomial);
        Polynomial.Combine(
            in planarRadius,
            1,
            in tempPolynomial,
            -1,
            out var signedRadiusSquared);

        RadialPolynomial.SetVariable(out var radial);
        Polynomial.Scale(in delta, a[0], out tempPolynomial);
        RadialPolynomial.SetConstant(in tempPolynomial, out var tempRadial);
        RadialPolynomial.Combine(
            in radial,
            1,
            in tempRadial,
            -1,
            out var h);
        Polynomial.Combine(in z, 1, in delta, -d[0], out tempPolynomial);
        RadialPolynomial.SetConstant(in tempPolynomial, out var height);
        RadialPolynomial.Combine(
            in h,
            a[2],
            in height,
            -d[3],
            out var armCos);
        RadialPolynomial.Combine(
            in h,
            d[3],
            in height,
            a[2],
            out var armSin);

        RadialPolynomial.Square(in h, out var twiceReach);
        RadialPolynomial.Square(in height, out tempRadial);
        RadialPolynomial.AddScaled(ref twiceReach, in tempRadial, 1);
        Polynomial.Scale(
            in deltaSquared,
            a[2] * a[2] + d[3] * d[3] - a[1] * a[1],
            out tempPolynomial);
        RadialPolynomial.AddConstant(ref twiceReach, in tempPolynomial);

        Polynomial.Multiply(in x, in y5X, out var dot);
        Polynomial.Multiply(in y, in y5Y, out tempPolynomial);
        Polynomial.AddScaled(ref dot, in tempPolynomial, 1);
        Polynomial.Multiply(in x, in y5Y, out var cross);
        Polynomial.Multiply(in y, in y5X, out tempPolynomial);
        Polynomial.AddScaled(ref cross, in tempPolynomial, -1);
        RadialPolynomial.Multiply(in radial, in dot, out var n);
        Polynomial.Multiply(in cross, in delta, out tempPolynomial);
        Polynomial.ScaleInPlace(ref tempPolynomial, d[2]);
        RadialPolynomial.AddConstant(ref n, in tempPolynomial);

        RadialPolynomial.Multiply(in armCos, in n, out var armOrientation);
        Polynomial.Multiply(in vertical, in planarRadius, out tempPolynomial);
        RadialPolynomial.Multiply(in armSin, in tempPolynomial, out tempRadial);
        RadialPolynomial.AddScaled(ref armOrientation, in tempRadial, 1);

        RadialPolynomial.Square(in n, out var orientationNorm);
        Polynomial.Square(in vertical, out tempPolynomial);
        Polynomial.Square(in planarRadius, out var termPolynomial);
        Polynomial.Multiply(
            in tempPolynomial,
            in termPolynomial,
            out var constantPolynomial);
        RadialPolynomial.AddConstant(ref orientationNorm, in constantPolynomial);

        RadialPolynomial.Square(in armOrientation, out tempRadial);
        Polynomial.Scale(in deltaSquared, 4, out tempPolynomial);
        RadialPolynomial.Multiply(in tempRadial, in tempPolynomial, out var equation);
        RadialPolynomial.Square(in twiceReach, out tempRadial);
        RadialPolynomial.Multiply(
            in tempRadial,
            in orientationNorm,
            out var termRadial);
        RadialPolynomial.AddScaled(ref equation, in termRadial, -1);

        ReduceRadius(
            in equation,
            in signedRadiusSquared,
            out var even,
            out var odd);

        Polynomial.Square(in even, out var resultant);
        Polynomial.Square(in odd, out tempPolynomial);
        Polynomial.Multiply(
            in signedRadiusSquared,
            in tempPolynomial,
            out termPolynomial);
        Polynomial.AddScaled(ref resultant, in termPolynomial, -1);

        Polynomial.Square(in planarRadius, out tempPolynomial);
        Polynomial.Square(in deltaSquared, out termPolynomial);
        Polynomial.Square(in termPolynomial, out constantPolynomial);
        Polynomial.Multiply(
            in tempPolynomial,
            in constantPolynomial,
            out var factor);

        if (!Polynomial.TryDivide(in resultant, in factor, out var quotient))
            return false;

        Polynomial.Multiply(in factor, in quotient, out tempPolynomial);
        Polynomial.Combine(
            in resultant,
            1,
            in tempPolynomial,
            -1,
            out var remainder);
        double scale = Math.Max(
            resultant.NormInfinity,
            factor.NormOne * quotient.NormInfinity);
        double relativeRemainder = remainder.NormInfinity / Math.Max(scale, 1e-300);

        if (!double.IsFinite(relativeRemainder)
            || relativeRemainder > MaxRemainder
            || quotient.EffectiveDegree > MaxDegree)
        {
            return false;
        }

        quotient.Normalize();

        if (!quotient.IsFinite)
            return false;

        int degree = quotient.CopyEffectiveTo(coefficients);
        result = new(degree, relativeRemainder);

        return true;
    }

    static void BuildComponent(
        double rx,
        double ry,
        double rz,
        double position,
        double a5,
        double d5,
        double d6,
        in Polynomial delta,
        in Polynomial cosNumerator,
        in Polynomial sinNumerator,
        out Polynomial y5,
        out Polynomial p4)
    {
        Polynomial.Combine(
            in cosNumerator,
            rx,
            in sinNumerator,
            -ry,
            out var x5);
        Polynomial.Combine(
            in sinNumerator,
            rx,
            in cosNumerator,
            ry,
            out y5);
        Polynomial.Scale(in delta, position - d6 * rz, out p4);
        Polynomial.AddScaled(ref p4, in x5, -a5);
        Polynomial.AddScaled(ref p4, in y5, -d5);
    }

    static bool AreFinite(ReadOnlySpan<double> values)
    {
        foreach (double value in values)
        {
            if (!double.IsFinite(value))
                return false;
        }

        return true;
    }

    static bool IsFinite(Transform value) =>
        double.IsFinite(value.M00)
        && double.IsFinite(value.M01)
        && double.IsFinite(value.M02)
        && double.IsFinite(value.M03)
        && double.IsFinite(value.M10)
        && double.IsFinite(value.M11)
        && double.IsFinite(value.M12)
        && double.IsFinite(value.M13)
        && double.IsFinite(value.M20)
        && double.IsFinite(value.M21)
        && double.IsFinite(value.M22)
        && double.IsFinite(value.M23);

    static void ReduceRadius(
        in RadialPolynomial value,
        in Polynomial signedRadiusSquared,
        out Polynomial even,
        out Polynomial odd)
    {
        Polynomial.Create([1], out var one);
        Polynomial.Square(in signedRadiusSquared, out var squared);
        Polynomial.Multiply(
            in squared,
            in signedRadiusSquared,
            out var cubed);
        even = default;
        odd = default;

        for (int i = 0; i <= value.Degree; i++)
        {
            ref Polynomial destination = ref ((i & 1) == 0 ? ref even : ref odd);

            switch (i / 2)
            {
                case 0:
                    RadialPolynomial.AddCoefficientProduct(
                        in value,
                        i,
                        in one,
                        ref destination);
                    break;
                case 1:
                    RadialPolynomial.AddCoefficientProduct(
                        in value,
                        i,
                        in signedRadiusSquared,
                        ref destination);
                    break;
                case 2:
                    RadialPolynomial.AddCoefficientProduct(
                        in value,
                        i,
                        in squared,
                        ref destination);
                    break;
                case 3:
                    RadialPolynomial.AddCoefficientProduct(
                        in value,
                        i,
                        in cubed,
                        ref destination);
                    break;
                default:
                    throw new InvalidOperationException("The radial degree exceeds the supported eliminant bound.");
            }
        }

        even.Trim();
        odd.Trim();
    }

    static double SnapTrig(double value) => Math.Abs(value) < 1e-15 ? 0 : value;

    [InlineArray(PolynomialCapacity)]
    struct PolynomialBuffer
    {
        double _element;
    }

    [InlineArray(MaxRadialDegree + 1)]
    struct RadialDegreeBuffer
    {
        int _element;
    }

    [InlineArray(RadialCapacity)]
    struct RadialBuffer
    {
        double _element;
    }

    struct RadialPolynomial
    {
        RadialBuffer _coefficients;
        RadialDegreeBuffer _tangentDegrees;
        public int Degree { readonly get; set; }

        public static void SetVariable(out RadialPolynomial result)
        {
            result = default;
            result.Degree = 1;
            result._coefficients[RadialStride] = 1;
        }

        public static void SetConstant(
            in Polynomial value,
            out RadialPolynomial result)
        {
            if (value.Degree > MaxRadialTangentDegree)
                throw new InvalidOperationException("The tangent degree exceeds the radial-polynomial bound.");

            result = default;
            result._tangentDegrees[0] = value.Degree;

            for (int i = 0; i <= value.Degree; i++)
                result._coefficients[i] = value[i];
        }

        public static void Combine(
            in RadialPolynomial left,
            double leftScale,
            in RadialPolynomial right,
            double rightScale,
            out RadialPolynomial result)
        {
            result = default;
            AddScaled(ref result, in left, leftScale);
            AddScaled(ref result, in right, rightScale);
        }

        public static void AddScaled(
            ref RadialPolynomial result,
            in RadialPolynomial value,
            double scale)
        {
            if (scale == 0)
                return;

            result.Degree = Math.Max(result.Degree, value.Degree);

            for (int radial = 0; radial <= value.Degree; radial++)
            {
                int tangentDegree = value._tangentDegrees[radial];
                result._tangentDegrees[radial] = Math.Max(
                    result._tangentDegrees[radial],
                    tangentDegree);
                int offset = radial * RadialStride;

                for (int tangent = 0; tangent <= tangentDegree; tangent++)
                {
                    result._coefficients[offset + tangent] +=
                        scale * value._coefficients[offset + tangent];
                }

                result.TrimCoefficient(radial);
            }

            result.Trim();
        }

        public static void AddConstant(
            ref RadialPolynomial result,
            in Polynomial value)
        {
            if (value.Degree > MaxRadialTangentDegree)
                throw new InvalidOperationException("The tangent degree exceeds the radial-polynomial bound.");

            result._tangentDegrees[0] = Math.Max(
                result._tangentDegrees[0],
                value.Degree);

            for (int i = 0; i <= value.Degree; i++)
                result._coefficients[i] += value[i];

            result.TrimCoefficient(0);
            result.Trim();
        }

        public static void Square(
            in RadialPolynomial value,
            out RadialPolynomial result) =>
            Multiply(in value, in value, out result);

        public static void Multiply(
            in RadialPolynomial left,
            in RadialPolynomial right,
            out RadialPolynomial result)
        {
            int degree = left.Degree + right.Degree;

            if (degree > MaxRadialDegree)
                throw new InvalidOperationException("The radial degree exceeds the supported eliminant bound.");

            result = default;
            result.Degree = degree;

            for (int leftRadial = 0; leftRadial <= left.Degree; leftRadial++)
            {
                int leftTangentDegree = left._tangentDegrees[leftRadial];
                int leftOffset = leftRadial * RadialStride;

                for (int rightRadial = 0; rightRadial <= right.Degree; rightRadial++)
                {
                    int rightTangentDegree = right._tangentDegrees[rightRadial];
                    int tangentDegree = leftTangentDegree + rightTangentDegree;

                    if (tangentDegree > MaxRadialTangentDegree)
                    {
                        throw new InvalidOperationException(
                            "The tangent degree exceeds the radial-polynomial bound.");
                    }

                    int radial = leftRadial + rightRadial;
                    int rightOffset = rightRadial * RadialStride;
                    int resultOffset = radial * RadialStride;
                    result._tangentDegrees[radial] = Math.Max(
                        result._tangentDegrees[radial],
                        tangentDegree);

                    for (int i = 0; i <= leftTangentDegree; i++)
                    {
                        double coefficient = left._coefficients[leftOffset + i];

                        for (int j = 0; j <= rightTangentDegree; j++)
                        {
                            result._coefficients[resultOffset + i + j] +=
                                coefficient * right._coefficients[rightOffset + j];
                        }
                    }
                }
            }

            for (int radial = 0; radial <= degree; radial++)
                result.TrimCoefficient(radial);

            result.Trim();
        }

        public static void Multiply(
            in RadialPolynomial left,
            in Polynomial right,
            out RadialPolynomial result)
        {
            result = default;
            result.Degree = left.Degree;

            for (int radial = 0; radial <= left.Degree; radial++)
            {
                int leftDegree = left._tangentDegrees[radial];
                int degree = leftDegree + right.Degree;

                if (degree > MaxRadialTangentDegree)
                {
                    throw new InvalidOperationException(
                        "The tangent degree exceeds the radial-polynomial bound.");
                }

                int offset = radial * RadialStride;
                result._tangentDegrees[radial] = degree;

                for (int i = 0; i <= leftDegree; i++)
                {
                    double coefficient = left._coefficients[offset + i];

                    for (int j = 0; j <= right.Degree; j++)
                        result._coefficients[offset + i + j] += coefficient * right[j];
                }

                result.TrimCoefficient(radial);
            }

            result.Trim();
        }

        public static void AddCoefficientProduct(
            in RadialPolynomial value,
            int radial,
            in Polynomial factor,
            ref Polynomial result)
        {
            int tangentDegree = value._tangentDegrees[radial];
            int degree = tangentDegree + factor.Degree;
            result.EnsureDegree(degree);
            int offset = radial * RadialStride;

            for (int i = 0; i <= tangentDegree; i++)
            {
                double coefficient = value._coefficients[offset + i];

                for (int j = 0; j <= factor.Degree; j++)
                    result[i + j] += coefficient * factor[j];
            }
        }

        void TrimCoefficient(int radial)
        {
            int degree = _tangentDegrees[radial];
            int offset = radial * RadialStride;

            while (degree > 0 && _coefficients[offset + degree] == 0)
                degree--;

            for (int i = degree + 1; i <= _tangentDegrees[radial]; i++)
                _coefficients[offset + i] = 0;

            _tangentDegrees[radial] = degree;
        }

        void Trim()
        {
            while (Degree > 0 && IsZero(Degree))
            {
                int offset = Degree * RadialStride;

                for (int i = 0; i < RadialStride; i++)
                    _coefficients[offset + i] = 0;

                _tangentDegrees[Degree] = 0;
                Degree--;
            }
        }

        readonly bool IsZero(int radial)
        {
            int degree = _tangentDegrees[radial];
            int offset = radial * RadialStride;

            for (int i = 0; i <= degree; i++)
            {
                if (_coefficients[offset + i] != 0)
                    return false;
            }

            return true;
        }
    }

    struct Polynomial
    {
        PolynomialBuffer _coefficients;
        public int Degree { readonly get; set; }

        public readonly int EffectiveDegree
        {
            get
            {
                double tolerance = DegreeTolerance * NormInfinity;
                int degree = Degree;

                while (degree > 0 && Math.Abs(_coefficients[degree]) <= tolerance)
                    degree--;

                return degree;
            }
        }

        public readonly double NormInfinity
        {
            get
            {
                double norm = 0;

                for (int i = 0; i <= Degree; i++)
                    norm = Math.Max(norm, Math.Abs(_coefficients[i]));

                return norm;
            }
        }

        public readonly double NormOne
        {
            get
            {
                double norm = 0;

                for (int i = 0; i <= Degree; i++)
                    norm += Math.Abs(_coefficients[i]);

                return norm;
            }
        }

        public readonly bool IsFinite
        {
            get
            {
                for (int i = 0; i <= Degree; i++)
                {
                    if (!double.IsFinite(_coefficients[i]))
                        return false;
                }

                return true;
            }
        }

        public double this[int index]
        {
            readonly get => index <= Degree ? _coefficients[index] : 0;
            set => _coefficients[index] = value;
        }

        public static void Create(
            ReadOnlySpan<double> coefficients,
            out Polynomial result)
        {
            if (coefficients.IsEmpty)
                throw new ArgumentException("At least one polynomial coefficient is required.", nameof(coefficients));

            if (coefficients.Length > PolynomialCapacity)
                throw new ArgumentException("The polynomial exceeds the coefficient buffer.", nameof(coefficients));

            result = default;
            result.Degree = coefficients.Length - 1;

            for (int i = 0; i < coefficients.Length; i++)
                result._coefficients[i] = coefficients[i];

            result.Trim();
        }

        public static void Combine(
            in Polynomial left,
            double leftScale,
            in Polynomial right,
            double rightScale,
            out Polynomial result)
        {
            result = default;
            AddScaled(ref result, in left, leftScale);
            AddScaled(ref result, in right, rightScale);
        }

        public static void Scale(
            in Polynomial value,
            double scale,
            out Polynomial result)
        {
            result = default;
            result.Degree = value.Degree;

            for (int i = 0; i <= value.Degree; i++)
                result._coefficients[i] = scale * value._coefficients[i];

            result.Trim();
        }

        public static void ScaleInPlace(ref Polynomial value, double scale)
        {
            for (int i = 0; i <= value.Degree; i++)
                value._coefficients[i] *= scale;

            value.Trim();
        }

        public static void AddScaled(
            ref Polynomial result,
            in Polynomial value,
            double scale)
        {
            if (scale == 0)
                return;

            result.EnsureDegree(value.Degree);

            for (int i = 0; i <= value.Degree; i++)
                result._coefficients[i] += scale * value._coefficients[i];

            result.Trim();
        }

        public static void Square(
            in Polynomial value,
            out Polynomial result) =>
            Multiply(in value, in value, out result);

        public static void Multiply(
            in Polynomial left,
            in Polynomial right,
            out Polynomial result)
        {
            int degree = left.Degree + right.Degree;

            if (degree >= PolynomialCapacity)
                throw new InvalidOperationException("The polynomial exceeds the coefficient buffer.");

            result = default;
            result.Degree = degree;

            for (int i = 0; i <= left.Degree; i++)
            {
                double coefficient = left._coefficients[i];

                for (int j = 0; j <= right.Degree; j++)
                    result._coefficients[i + j] += coefficient * right._coefficients[j];
            }

            result.Trim();
        }

        public static bool TryDivide(
            in Polynomial dividend,
            in Polynomial divisor,
            out Polynomial quotient)
        {
            int dividendDegree = dividend.EffectiveDegree;
            int divisorDegree = divisor.EffectiveDegree;
            double divisorLeading = divisor._coefficients[divisorDegree];

            if (Math.Abs(divisorLeading) <= DegreeTolerance * divisor.NormInfinity)
            {
                quotient = default;
                return false;
            }

            quotient = default;

            if (dividendDegree < divisorDegree)
                return true;

            Span<double> remainder = stackalloc double[PolynomialCapacity];

            for (int i = 0; i <= dividendDegree; i++)
                remainder[i] = dividend._coefficients[i];

            quotient.Degree = dividendDegree - divisorDegree;

            for (int i = quotient.Degree; i >= 0; i--)
            {
                double factor = remainder[divisorDegree + i] / divisorLeading;
                quotient._coefficients[i] = factor;

                for (int j = 0; j <= divisorDegree; j++)
                    remainder[i + j] -= factor * divisor._coefficients[j];
            }

            quotient.Trim();
            return true;
        }

        public void Normalize()
        {
            double norm = NormInfinity;

            if (norm is not (0 or 1))
            {
                for (int i = 0; i <= Degree; i++)
                    _coefficients[i] /= norm;
            }
        }

        public readonly int CopyEffectiveTo(Span<double> destination)
        {
            int degree = EffectiveDegree;

            if (destination.Length <= degree)
                throw new ArgumentException("The coefficient buffer is too small.", nameof(destination));

            for (int i = 0; i <= degree; i++)
                destination[i] = _coefficients[i];

            return degree;
        }

        public void EnsureDegree(int degree)
        {
            if (degree >= PolynomialCapacity)
                throw new InvalidOperationException("The polynomial exceeds the coefficient buffer.");

            Degree = Math.Max(Degree, degree);
        }

        public void Trim()
        {
            int degree = Degree;

            while (degree > 0 && _coefficients[degree] == 0)
                degree--;

            for (int i = degree + 1; i <= Degree; i++)
                _coefficients[i] = 0;

            Degree = degree;
        }
    }
}
