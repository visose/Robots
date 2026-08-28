namespace Robots;

static class PolynomialRoots
{
    const int MaxDegree = 16;
    const int Stride = MaxDegree + 1;
    const double ZeroTolerance = 1e-12;

    /// <summary>
    /// Finds distinct real roots with an allocation-free derivative partition.
    /// <see cref="ExactPolynomialRoots"/> handles inputs this bounded path rejects.
    /// </summary>
    public static bool TryFind(
        ReadOnlySpan<double> coefficients,
        Span<double> roots,
        out int rootCount) =>
        TryFind(
            coefficients,
            roots,
            out rootCount,
            [],
            out _);

    /// <summary>
    /// Finds roots and stationary points in one pass.
    /// </summary>
    public static bool TryFind(
        ReadOnlySpan<double> coefficients,
        Span<double> roots,
        out int rootCount,
        Span<double> stationaryRoots,
        out int stationaryRootCount)
    {
        rootCount = 0;
        stationaryRootCount = 0;

        if (coefficients.IsEmpty)
            throw new ArgumentException("At least one polynomial coefficient is required.", nameof(coefficients));

        int degree = coefficients.Length - 1;

        while (degree >= 0 && coefficients[degree] == 0)
            degree--;

        if (degree < 0)
            throw new ArgumentException("The zero polynomial has an indeterminate set of roots.", nameof(coefficients));

        if (degree > MaxDegree)
            throw new ArgumentException($"The polynomial degree must not exceed {MaxDegree}.", nameof(coefficients));

        if (roots.Length < degree)
            throw new ArgumentException("The root buffer is too small for the polynomial degree.", nameof(roots));

        if (degree == 0)
            return double.IsFinite(coefficients[0]);

        Span<double> polynomials = stackalloc double[Stride * Stride];
        Span<double> derivativeRoots = stackalloc double[Stride * Stride];
        Span<int> rootCounts = stackalloc int[Stride];
        polynomials.Clear();
        derivativeRoots.Clear();
        var polynomial = GetWorkspace(polynomials, degree);
        coefficients[..(degree + 1)].CopyTo(polynomial);

        if (!TryNormalize(polynomial, degree))
            return false;

        for (int currentDegree = degree; currentDegree > 1; currentDegree--)
        {
            var current = GetWorkspace(polynomials, currentDegree);
            var derivative = GetWorkspace(polynomials, currentDegree - 1);

            for (int i = 1; i <= currentDegree; i++)
                derivative[i - 1] = i * current[i];

            if (!TryNormalize(derivative, currentDegree - 1))
                return false;
        }

        for (int currentDegree = 1; currentDegree <= degree; currentDegree++)
        {
            var current = GetWorkspace(polynomials, currentDegree);
            var currentRoots = GetWorkspace(derivativeRoots, currentDegree);
            int currentRootCount;

            if (currentDegree == 1)
            {
                double root = -current[0] / current[1];
                currentRootCount = double.IsFinite(root) && -1 <= root && root <= 1 ? 1 : 0;

                if (currentRootCount == 1)
                    currentRoots[0] = root;
            }
            else
            {
                var criticalPoints = GetWorkspace(derivativeRoots, currentDegree - 1);
                int criticalPointCount = rootCounts[currentDegree - 1];
                currentRootCount = FindRoots(
                    current,
                    currentDegree,
                    criticalPoints[..criticalPointCount],
                    currentRoots);
            }

            if (currentRootCount < 0 || currentRootCount > currentDegree)
                return false;

            rootCounts[currentDegree] = currentRootCount;
        }

        var result = GetWorkspace(derivativeRoots, degree);
        rootCount = rootCounts[degree];
        result[..rootCount].CopyTo(roots);

        if (degree > 1)
        {
            var stationaryResult = GetWorkspace(derivativeRoots, degree - 1);
            stationaryRootCount = rootCounts[degree - 1];

            if (!stationaryRoots.IsEmpty)
            {
                if (stationaryRoots.Length < stationaryRootCount)
                    throw new ArgumentException("The stationary-root buffer is too small.", nameof(stationaryRoots));

                stationaryResult[..stationaryRootCount].CopyTo(stationaryRoots);
            }
        }

        return true;
    }

    static int FindRoots(
        ReadOnlySpan<double> polynomial,
        int degree,
        ReadOnlySpan<double> criticalPoints,
        Span<double> roots)
    {
        int rootCount = 0;
        double left = -1;
        double leftValue = Evaluate(polynomial, degree, left);

        if (!double.IsFinite(leftValue))
            return -1;

        if (IsNearZero(polynomial, degree, left, leftValue))
            roots[rootCount++] = left;

        for (int i = 0; i <= criticalPoints.Length; i++)
        {
            double right = i < criticalPoints.Length ? criticalPoints[i] : 1;
            double rightValue = Evaluate(polynomial, degree, right);

            if (!double.IsFinite(rightValue))
                return -1;

            if (OppositeSigns(leftValue, rightValue))
            {
                double root = RefineBracket(polynomial, degree, left, right, leftValue, rightValue);

                if (!double.IsFinite(root))
                    return -1;

                if (!TryAddRoot(roots, ref rootCount, root))
                    return -1;
            }

            if (IsNearZero(polynomial, degree, right, rightValue))
            {
                if (!TryAddRoot(roots, ref rootCount, right))
                    return -1;
            }

            left = right;
            leftValue = rightValue;
        }

        return rootCount;
    }

    static double RefineBracket(
        ReadOnlySpan<double> polynomial,
        int degree,
        double left,
        double right,
        double leftValue,
        double rightValue)
    {
        for (int iteration = 0; iteration < 64; iteration++)
        {
            double midpoint = left + (right - left) * 0.5;

            if (midpoint == left || midpoint == right)
                break;

            double midpointValue = Evaluate(polynomial, degree, midpoint);

            if (!double.IsFinite(midpointValue))
                return double.NaN;

            if (midpointValue == 0)
                return midpoint;

            if (OppositeSigns(leftValue, midpointValue))
            {
                right = midpoint;
                rightValue = midpointValue;
            }
            else
            {
                left = midpoint;
                leftValue = midpointValue;
            }
        }

        return RelativeError(polynomial, degree, left, leftValue)
            <= RelativeError(polynomial, degree, right, rightValue)
                ? left
                : right;
    }

    static double RelativeError(
        ReadOnlySpan<double> polynomial,
        int degree,
        double value,
        double result)
    {
        double scale = ScaleAt(polynomial, degree, value);
        return Math.Abs(result) / Math.Max(scale, double.Epsilon);
    }

    static bool IsNearZero(
        ReadOnlySpan<double> polynomial,
        int degree,
        double value,
        double result) =>
        RelativeError(polynomial, degree, value, result) <= ZeroTolerance;

    static double ScaleAt(ReadOnlySpan<double> polynomial, int degree, double value)
    {
        double absoluteValue = Math.Abs(value);
        double scale = Math.Abs(polynomial[degree]);

        for (int i = degree - 1; i >= 0; i--)
            scale = scale * absoluteValue + Math.Abs(polynomial[i]);

        return scale;
    }

    static double Evaluate(ReadOnlySpan<double> polynomial, int degree, double value)
    {
        double result = polynomial[degree];

        for (int i = degree - 1; i >= 0; i--)
            result = result * value + polynomial[i];

        return result;
    }

    static bool TryNormalize(Span<double> polynomial, int degree)
    {
        double scale = 0;

        for (int i = 0; i <= degree; i++)
        {
            if (!double.IsFinite(polynomial[i]))
                return false;

            scale = Math.Max(scale, Math.Abs(polynomial[i]));
        }

        if (scale == 0)
            return false;

        for (int i = 0; i <= degree; i++)
            polynomial[i] /= scale;

        return true;
    }

    static bool OppositeSigns(double first, double second) =>
        first < 0 && second > 0 || first > 0 && second < 0;

    static bool TryAddRoot(Span<double> roots, ref int count, double root)
    {
        if (count > 0 && Math.Abs(root - roots[count - 1]) <= ZeroTolerance)
            return false;

        if (count == roots.Length)
            return false;

        roots[count++] = root;
        return true;
    }

    static Span<double> GetWorkspace(Span<double> workspace, int degree) =>
        workspace.Slice(degree * Stride, Stride);
}
