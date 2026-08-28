using System.Numerics;

namespace Robots;

/// <summary>
/// Finds the distinct real roots of a polynomial in the interval [-1, 1].
/// </summary>
static class ExactPolynomialRoots
{
    const int MaxDegree = 16;
    const int TargetDepth = 58;
    // Binary64 roots can be separated much more deeply near zero than near one.
    const int MaxDepth = 1100;

    /// <summary>
    /// Finds distinct real roots of binary64 coefficients exactly, in ascending order.
    /// </summary>
    public static double[] Find(ReadOnlySpan<double> coefficients)
    {
        if (!TryFind(coefficients, out var roots))
            throw new InvalidOperationException("Distinct polynomial roots cannot be represented separately at double precision.");

        return roots;
    }

    /// <summary>
    /// Tries to find distinct real roots exactly, returning false only when
    /// distinct roots cannot be represented separately at double precision.
    /// </summary>
    public static bool TryFind(
        ReadOnlySpan<double> coefficients,
        out double[] roots)
    {
        var polynomial = IntegerPolynomial.FromDoubles(coefficients);

        if (polynomial.IsZero)
            throw new ArgumentException("The zero polynomial has an indeterminate set of roots.", nameof(coefficients));

        if (polynomial.Degree > MaxDegree)
            throw new ArgumentException($"The polynomial degree must not exceed {MaxDegree}.", nameof(coefficients));

        if (polynomial.IsConstant)
        {
            roots = [];
            return true;
        }

        var chain = CreateSturmChain(ref polynomial);
        var left = new Dyadic(-BigInteger.One, 0);
        var right = new Dyadic(BigInteger.One, 0);
        var leftVariations = CountVariations(chain, left);
        var rightVariations = CountVariations(chain, right);
        var found = new List<double>(polynomial.Degree);

        // V(a) - V(b) counts roots in (a, b], so include the left endpoint explicitly.
        if (polynomial.SignAt(left) == 0)
            found.Add(-1);

        var pending = new Stack<Interval>(TargetDepth + 1);
        pending.Push(new(left, right, leftVariations, rightVariations, 0));

        while (pending.TryPop(out var interval))
        {
            var count = interval.LeftVariations - interval.RightVariations;

            if (count == 0)
                continue;

            if (count < 0)
                throw new InvalidOperationException("The Sturm sequence produced an invalid root count.");

            if (count == 1 && interval.Depth >= TargetDepth)
            {
                found.Add(Dyadic.Midpoint(interval.Left, interval.Right).ToDouble());
                continue;
            }

            if (interval.Depth >= MaxDepth)
            {
                roots = [];
                return false;
            }

            var midpoint = Dyadic.Midpoint(interval.Left, interval.Right);
            var midpointVariations = CountVariations(chain, midpoint);
            var nextDepth = interval.Depth + 1;

            // Push the right interval first so the deterministic traversal remains left-to-right.
            pending.Push(new(midpoint, interval.Right, midpointVariations, interval.RightVariations, nextDepth));
            pending.Push(new(interval.Left, midpoint, interval.LeftVariations, midpointVariations, nextDepth));
        }

        found.Sort();

        for (int i = 1; i < found.Count; i++)
        {
            if (found[i] == found[i - 1])
            {
                roots = [];
                return false;
            }
        }

        roots = [.. found];
        return true;
    }

    static IntegerPolynomial[] CreateSturmChain(ref IntegerPolynomial polynomial)
    {
        polynomial = polynomial.Primitive(positiveLeading: true);
        var chain = new List<IntegerPolynomial>(polynomial.Degree + 1)
        {
            polynomial,
            polynomial.Derivative().Primitive()
        };

        while (!chain[^1].IsConstant)
        {
            var remainder = Remainder(chain[^2], chain[^1]);

            if (remainder.IsZero)
            {
                polynomial = DivideExact(polynomial, chain[^1])
                    .Primitive(positiveLeading: true);

                return CreateSquareFreeChain(polynomial);
            }

            chain.Add(remainder.Negate());
        }

        polynomial = chain[0];
        return [.. chain];
    }

    static IntegerPolynomial[] CreateSquareFreeChain(IntegerPolynomial polynomial)
    {
        var chain = new List<IntegerPolynomial>(polynomial.Degree + 1)
        {
            polynomial,
            polynomial.Derivative().Primitive()
        };

        while (!chain[^1].IsConstant)
        {
            var remainder = Remainder(chain[^2], chain[^1]);

            if (remainder.IsZero)
                throw new InvalidOperationException("The square-free polynomial produced a degenerate Sturm sequence.");

            chain.Add(remainder.Negate());
        }

        return [.. chain];
    }

    static int CountVariations(IntegerPolynomial[] chain, Dyadic value)
    {
        int previous = 0;
        int variations = 0;

        foreach (var polynomial in chain)
        {
            var sign = polynomial.SignAt(value);

            if (sign == 0)
                continue;

            if (previous != 0 && sign != previous)
                variations++;

            previous = sign;
        }

        return variations;
    }

    static IntegerPolynomial Remainder(IntegerPolynomial dividend, IntegerPolynomial divisor)
    {
        if (divisor.IsZero)
            throw new DivideByZeroException();

        if (dividend.Degree < divisor.Degree)
            return dividend.Primitive();

        var work = dividend.ToRationals();
        var divisorCoefficients = divisor.ToRationals();
        var leading = divisorCoefficients[^1];

        for (int degree = dividend.Degree; degree >= divisor.Degree; degree--)
        {
            if (work[degree].IsZero)
                continue;

            var factor = work[degree] / leading;
            var offset = degree - divisor.Degree;

            for (int i = 0; i <= divisor.Degree; i++)
                work[offset + i] -= factor * divisorCoefficients[i];
        }

        return IntegerPolynomial.FromRationals(work.AsSpan(0, divisor.Degree));
    }

    static IntegerPolynomial DivideExact(IntegerPolynomial dividend, IntegerPolynomial divisor)
    {
        if (divisor.IsZero)
            throw new DivideByZeroException();

        if (dividend.Degree < divisor.Degree)
            throw new InvalidOperationException("The polynomial division is not exact.");

        var work = dividend.ToRationals();
        var quotient = new Rational[dividend.Degree - divisor.Degree + 1];
        var divisorCoefficients = divisor.ToRationals();
        var leading = divisorCoefficients[^1];

        for (int degree = dividend.Degree; degree >= divisor.Degree; degree--)
        {
            if (work[degree].IsZero)
                continue;

            var factor = work[degree] / leading;
            var offset = degree - divisor.Degree;
            quotient[offset] = factor;

            for (int i = 0; i <= divisor.Degree; i++)
                work[offset + i] -= factor * divisorCoefficients[i];
        }

        for (int i = 0; i < divisor.Degree; i++)
        {
            if (!work[i].IsZero)
                throw new InvalidOperationException("The polynomial division is not exact.");
        }

        return IntegerPolynomial.FromRationals(quotient);
    }

    readonly record struct Interval(
        Dyadic Left,
        Dyadic Right,
        int LeftVariations,
        int RightVariations,
        int Depth);

    readonly record struct Dyadic(BigInteger Numerator, int Exponent)
    {
        public static Dyadic Midpoint(Dyadic first, Dyadic second)
        {
            var exponent = Math.Max(first.Exponent, second.Exponent);
            var firstNumerator = first.Numerator << (exponent - first.Exponent);
            var secondNumerator = second.Numerator << (exponent - second.Exponent);
            return new(firstNumerator + secondNumerator, exponent + 1);
        }

        public double ToDouble() => Math.ScaleB((double)Numerator, -Exponent);
    }

    class IntegerPolynomial
    {
        readonly BigInteger[] _coefficients;

        IntegerPolynomial(BigInteger[] coefficients)
        {
            int length = coefficients.Length;

            while (length > 0 && coefficients[length - 1].IsZero)
                length--;

            _coefficients = length == coefficients.Length
                ? coefficients
                : coefficients.AsSpan(0, length).ToArray();
        }

        public int Degree => _coefficients.Length - 1;
        public bool IsZero => _coefficients.Length == 0;
        public bool IsConstant => Degree <= 0;
        public BigInteger LeadingCoefficient => _coefficients[^1];
        public BigInteger this[int index] => _coefficients[index];

        public static IntegerPolynomial FromDoubles(ReadOnlySpan<double> coefficients)
        {
            if (coefficients.IsEmpty)
                throw new ArgumentException("At least one polynomial coefficient is required.", nameof(coefficients));

            int length = coefficients.Length;

            while (length > 0 && coefficients[length - 1] == 0)
                length--;

            if (length == 0)
                return new([]);

            var minExponent = int.MaxValue;

            for (int i = 0; i < length; i++)
            {
                var value = coefficients[i];

                if (!double.IsFinite(value))
                    throw new ArgumentException("Polynomial coefficients must be finite.", nameof(coefficients));

                if (value == 0)
                    continue;

                Decompose(value, out _, out int exponent);
                minExponent = Math.Min(minExponent, exponent);
            }

            var integers = new BigInteger[length];

            for (int i = 0; i < length; i++)
            {
                if (coefficients[i] == 0)
                    continue;

                Decompose(coefficients[i], out var mantissa, out int exponent);
                integers[i] = mantissa << (exponent - minExponent);
            }

            return new IntegerPolynomial(integers).Primitive();
        }

        public static IntegerPolynomial FromRationals(ReadOnlySpan<Rational> coefficients)
        {
            int length = coefficients.Length;

            while (length > 0 && coefficients[length - 1].IsZero)
                length--;

            if (length == 0)
                return new([]);

            var commonDenominator = BigInteger.One;

            for (int i = 0; i < length; i++)
            {
                if (coefficients[i].IsZero)
                    continue;

                commonDenominator = LeastCommonMultiple(commonDenominator, coefficients[i].Denominator);
            }

            var integers = new BigInteger[length];

            for (int i = 0; i < length; i++)
            {
                if (!coefficients[i].IsZero)
                    integers[i] = coefficients[i].Numerator * (commonDenominator / coefficients[i].Denominator);
            }

            return new IntegerPolynomial(integers).Primitive();
        }

        public IntegerPolynomial Derivative()
        {
            if (Degree <= 0)
                return new([]);

            var derivative = new BigInteger[Degree];

            for (int i = 1; i < _coefficients.Length; i++)
                derivative[i - 1] = _coefficients[i] * i;

            return new(derivative);
        }

        public IntegerPolynomial Negate()
        {
            var coefficients = new BigInteger[_coefficients.Length];

            for (int i = 0; i < coefficients.Length; i++)
                coefficients[i] = -_coefficients[i];

            return new(coefficients);
        }

        public IntegerPolynomial Primitive(bool positiveLeading = false)
        {
            if (IsZero)
                return this;

            var content = BigInteger.Zero;

            foreach (var coefficient in _coefficients)
                content = BigInteger.GreatestCommonDivisor(content, BigInteger.Abs(coefficient));

            var sign = positiveLeading && LeadingCoefficient.Sign < 0 ? -BigInteger.One : BigInteger.One;

            if (content.IsOne && sign.IsOne)
                return this;

            var coefficients = new BigInteger[_coefficients.Length];

            for (int i = 0; i < coefficients.Length; i++)
                coefficients[i] = sign * _coefficients[i] / content;

            return new(coefficients);
        }

        public Rational[] ToRationals()
        {
            var coefficients = new Rational[_coefficients.Length];

            for (int i = 0; i < coefficients.Length; i++)
                coefficients[i] = new(_coefficients[i], BigInteger.One);

            return coefficients;
        }

        public int SignAt(Dyadic value)
        {
            if (IsZero)
                return 0;

            var result = LeadingCoefficient;

            for (int i = Degree - 1; i >= 0; i--)
            {
                int shift = value.Exponent * (Degree - i);
                result = result * value.Numerator + (_coefficients[i] << shift);
            }

            return result.Sign;
        }

        static BigInteger LeastCommonMultiple(BigInteger first, BigInteger second)
            => first / BigInteger.GreatestCommonDivisor(first, second) * second;

        static void Decompose(double value, out BigInteger mantissa, out int exponent)
        {
            var bits = unchecked((ulong)BitConverter.DoubleToInt64Bits(value));
            var biasedExponent = (int)((bits >> 52) & 0x7ff);
            var fraction = bits & 0x000f_ffff_ffff_ffff;
            ulong magnitude;

            if (biasedExponent == 0)
            {
                magnitude = fraction;
                exponent = -1074;
            }
            else
            {
                magnitude = fraction | 0x0010_0000_0000_0000;
                exponent = biasedExponent - 1023 - 52;
            }

            mantissa = (bits >> 63) == 0 ? magnitude : -new BigInteger(magnitude);
        }
    }

    readonly record struct Rational
    {
        public Rational(BigInteger numerator, BigInteger denominator)
        {
            if (denominator.IsZero)
                throw new DivideByZeroException();

            if (numerator.IsZero)
            {
                Numerator = BigInteger.Zero;
                Denominator = BigInteger.One;
                return;
            }

            if (denominator.Sign < 0)
            {
                numerator = -numerator;
                denominator = -denominator;
            }

            var divisor = BigInteger.GreatestCommonDivisor(BigInteger.Abs(numerator), denominator);
            Numerator = numerator / divisor;
            Denominator = denominator / divisor;
        }

        public BigInteger Numerator { get; }
        public BigInteger Denominator { get; }
        public bool IsZero => Numerator.IsZero;

        public static Rational operator -(Rational value)
        {
            return value.IsZero
                ? new(BigInteger.Zero, BigInteger.One)
                : new(-value.Numerator, value.Denominator);
        }

        public static Rational operator +(Rational first, Rational second)
        {
            if (first.IsZero)
                return second.IsZero ? new(BigInteger.Zero, BigInteger.One) : second;

            if (second.IsZero)
                return first;

            return new(
                first.Numerator * second.Denominator + second.Numerator * first.Denominator,
                first.Denominator * second.Denominator);
        }

        public static Rational operator -(Rational first, Rational second)
        {
            return first + -second;
        }

        public static Rational operator *(Rational first, Rational second)
        {
            if (first.IsZero || second.IsZero)
                return new(BigInteger.Zero, BigInteger.One);

            return new(first.Numerator * second.Numerator, first.Denominator * second.Denominator);
        }

        public static Rational operator /(Rational first, Rational second)
        {
            if (second.IsZero)
                throw new DivideByZeroException();

            if (first.IsZero)
                return new(BigInteger.Zero, BigInteger.One);

            return new(first.Numerator * second.Denominator, first.Denominator * second.Numerator);
        }
    }
}
