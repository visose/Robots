using Rhino.Geometry;
using static System.Math;
using static Robots.GeometryMath;

namespace Robots;

/// <summary>
/// Solves a general six-revolute inverse problem through the
/// Raghavan-Roth matrix polynomial.
/// </summary>
/// <remarks>
/// Each joint is supplied as Rz(theta) * K, where K is its rigid transform at theta zero.
/// </remarks>
static class RaghavanRoth
{
    const int JointCount = 6;
    const int EquationCount = 14;
    const int LeftBasisCount = 9;
    const int RightBasisCount = 8;
    const int AngleSampleCount = 3;
    const int LeftSampleCount = 27;
    const int RightSampleCount = 9;
    const int PolynomialSize = 12;
    const int PencilSize = PolynomialSize * 2;
    const int PSinOffset = 0;
    const int PCosOffset = LeftBasisCount * EquationCount;
    const int POneOffset = LeftBasisCount * EquationCount * 2;
    const int QOffset = LeftBasisCount * EquationCount * 3;
    const double RootTolerance = 1e-6;

    public const int SampleTransformCount = JointCount * 3;

    public readonly record struct Split(
        int Linear,
        int Left0,
        int Left1,
        int Right0,
        int Right1,
        int Drop);

    /// <summary>
    /// Adds the solutions found with one algebraic split.
    /// </summary>
    public static void Solve(
        ReadOnlySpan<Transform> factors,
        Transform target,
        Split split,
        ReadOnlySpan<Transform> sampleTransforms,
        ReadOnlySpan<Transform> inverseSampleTransforms,
        List<double[]> solutions)
    {
        if (factors.Length != JointCount)
            throw new ArgumentException("Six joint factors are required.", nameof(factors));

        if (sampleTransforms.Length != SampleTransformCount)
            throw new ArgumentException($"{SampleTransformCount} sampled transforms are required.", nameof(sampleTransforms));

        if (inverseSampleTransforms.Length != SampleTransformCount)
            throw new ArgumentException($"{SampleTransformCount} inverse sampled transforms are required.", nameof(inverseSampleTransforms));

        RaghavanRothCoefficientRecovery.Buffer coefficientBuffer = default;
        Span<double> coefficients = coefficientBuffer;
        BuildCoefficients(
            target,
            split,
            sampleTransforms,
            inverseSampleTransforms,
            coefficients);

        Span<double> reflectors = stackalloc double[RightBasisCount];
        Span<int> permutation = stackalloc int[RightBasisCount];

        if (!FactorQ(coefficients, reflectors, permutation))
            return;

        Span<double> polynomial = stackalloc double[PolynomialSize * PolynomialSize * 3];
        polynomial.Clear();
        BuildPolynomial(coefficients, reflectors, polynomial);
        SolvePolynomial(
            factors,
            target,
            split,
            coefficients,
            reflectors,
            permutation,
            polynomial,
            solutions);
    }

    public static void BuildSampleTransforms(
        ReadOnlySpan<Transform> factors,
        Span<Transform> transforms,
        Span<Transform> inverseTransforms)
    {
        ReadOnlySpan<double> cosineTheta =
        [
            1,
            -0.5,
            -0.5
        ];
        ReadOnlySpan<double> sineTheta =
        [
            0,
            0.8660254037844386,
            -0.8660254037844386
        ];

        if (factors.Length != JointCount)
            throw new ArgumentException("Six joint factors are required.", nameof(factors));

        if (transforms.Length != SampleTransformCount)
            throw new ArgumentException($"{SampleTransformCount} sampled transforms are required.", nameof(transforms));

        if (inverseTransforms.Length != SampleTransformCount)
            throw new ArgumentException($"{SampleTransformCount} inverse sampled transforms are required.", nameof(inverseTransforms));

        for (int joint = 0; joint < JointCount; joint++)
        {
            for (int sample = 0; sample < AngleSampleCount; sample++)
            {
                int index = sample * JointCount + joint;
                transforms[index] = JointTransform(
                    factors[joint],
                    cosineTheta[sample],
                    sineTheta[sample]);
                inverseTransforms[index] = RigidInverse(transforms[index]);
            }
        }
    }

    static void BuildCoefficients(
        Transform target,
        Split split,
        ReadOnlySpan<Transform> sampleTransforms,
        ReadOnlySpan<Transform> inverseSampleTransforms,
        Span<double> coefficients)
    {
        Span<double> left = stackalloc double[LeftSampleCount * EquationCount];
        Span<double> right = stackalloc double[RightSampleCount * EquationCount];
        BuildEquationFeatures(
            target,
            split,
            sampleTransforms,
            inverseSampleTransforms,
            left,
            right);

        for (int row = 0; row < RaghavanRothCoefficientRecovery.SampleCount; row++)
        {
            int leftIndex = row < LeftSampleCount ? row : 0;
            int rightIndex = row < LeftSampleCount ? 0 : row - LeftSampleCount + 1;
            var leftFeatures = left.Slice(leftIndex * EquationCount, EquationCount);
            var rightFeatures = right.Slice(rightIndex * EquationCount, EquationCount);

            for (int column = 0; column < EquationCount; column++)
            {
                coefficients[row * EquationCount + column] =
                    leftFeatures[column] - rightFeatures[column];
            }
        }

        RaghavanRothCoefficientRecovery.Recover(coefficients, coefficients);
    }

    static void BuildEquationFeatures(
        Transform target,
        Split split,
        ReadOnlySpan<Transform> sampleTransforms,
        ReadOnlySpan<Transform> inverseSampleTransforms,
        Span<double> left,
        Span<double> right)
    {
        Span<Transform> pairs = stackalloc Transform[RightSampleCount];

        for (int linear = 0; linear < AngleSampleCount; linear++)
        {
            for (int left0 = 0; left0 < AngleSampleCount; left0++)
            {
                pairs[linear * AngleSampleCount + left0] =
                    sampleTransforms[linear * JointCount + split.Linear]
                    * sampleTransforms[left0 * JointCount + split.Left0];
            }
        }

        for (int linear = 0; linear < AngleSampleCount; linear++)
        {
            for (int left0 = 0; left0 < AngleSampleCount; left0++)
            {
                var pair = pairs[linear * AngleSampleCount + left0];

                for (int left1 = 0; left1 < AngleSampleCount; left1++)
                {
                    int row = (linear * AngleSampleCount + left0) * AngleSampleCount + left1;
                    var transform = pair * sampleTransforms[left1 * JointCount + split.Left1];
                    BuildFeatures(transform, left.Slice(row * EquationCount, EquationCount));
                }
            }
        }

        var dropped = inverseSampleTransforms[split.Drop];
        Span<Transform> prefix = stackalloc Transform[AngleSampleCount];

        if (split.Linear == 0)
        {
            for (int right1 = 0; right1 < AngleSampleCount; right1++)
            {
                prefix[right1] = target
                    * inverseSampleTransforms[right1 * JointCount + split.Right1];
            }

            for (int right0 = 0; right0 < AngleSampleCount; right0++)
            {
                var middle = inverseSampleTransforms[right0 * JointCount + split.Right0];

                for (int right1 = 0; right1 < AngleSampleCount; right1++)
                {
                    int column = right0 * AngleSampleCount + right1;
                    var transform = prefix[right1] * middle * dropped;
                    BuildFeatures(transform, right.Slice(column * EquationCount, EquationCount));
                }
            }

            return;
        }

        if (split.Linear == 1)
        {
            for (int right0 = 0; right0 < AngleSampleCount; right0++)
            {
                prefix[right0] = inverseSampleTransforms[right0 * JointCount + split.Right0]
                    * target;
            }

            for (int right0 = 0; right0 < AngleSampleCount; right0++)
            {
                for (int right1 = 0; right1 < AngleSampleCount; right1++)
                {
                    int column = right0 * AngleSampleCount + right1;
                    var transform = prefix[right0]
                        * inverseSampleTransforms[right1 * JointCount + split.Right1]
                        * dropped;
                    BuildFeatures(transform, right.Slice(column * EquationCount, EquationCount));
                }
            }

            return;
        }

        for (int right0 = 0; right0 < AngleSampleCount; right0++)
        {
            for (int right1 = 0; right1 < AngleSampleCount; right1++)
            {
                int column = right0 * AngleSampleCount + right1;
                var transform = inverseSampleTransforms[right1 * JointCount + split.Right1]
                    * inverseSampleTransforms[right0 * JointCount + split.Right0]
                    * target
                    * dropped;
                BuildFeatures(transform, right.Slice(column * EquationCount, EquationCount));
            }
        }
    }

    static void BuildFeatures(Transform transform, Span<double> features)
    {
        double axisX = transform.M02;
        double axisY = transform.M12;
        double axisZ = transform.M22;
        double pointX = transform.M03;
        double pointY = transform.M13;
        double pointZ = transform.M23;
        double pointSquared = pointX * pointX + pointY * pointY + pointZ * pointZ;
        double projection = axisX * pointX + axisY * pointY + axisZ * pointZ;
        features[0] = axisX;
        features[1] = axisY;
        features[2] = axisZ;
        features[3] = pointX;
        features[4] = pointY;
        features[5] = pointZ;
        features[6] = axisY * pointZ - axisZ * pointY;
        features[7] = axisZ * pointX - axisX * pointZ;
        features[8] = axisX * pointY - axisY * pointX;
        features[9] = pointSquared;
        features[10] = projection;
        features[11] = axisX * pointSquared - 2 * projection * pointX;
        features[12] = axisY * pointSquared - 2 * projection * pointY;
        features[13] = axisZ * pointSquared - 2 * projection * pointZ;
    }

    static bool FactorQ(
        Span<double> coefficients,
        Span<double> reflectors,
        Span<int> permutation)
    {
        var q = coefficients[QOffset..];

        for (int i = 0; i < RightBasisCount; i++)
            permutation[i] = i;

        double maximumDiagonal = 0;

        for (int column = 0; column < RightBasisCount; column++)
        {
            int pivot = column;
            double pivotNorm = -1;

            for (int candidate = column; candidate < RightBasisCount; candidate++)
            {
                double norm = 0;

                for (int row = column; row < EquationCount; row++)
                {
                    double value = q[candidate * EquationCount + row];
                    norm += value * value;
                }

                if (norm > pivotNorm)
                {
                    pivotNorm = norm;
                    pivot = candidate;
                }
            }

            if (pivot != column)
            {
                for (int row = 0; row < EquationCount; row++)
                    Swap(ref q[column * EquationCount + row], ref q[pivot * EquationCount + row]);

                Swap(ref permutation[column], ref permutation[pivot]);
            }

            int diagonal = column * EquationCount + column;
            double first = q[diagonal];
            double tailNorm = 0;

            for (int row = column + 1; row < EquationCount; row++)
                tailNorm = Hypot(tailNorm, q[column * EquationCount + row]);

            if (tailNorm == 0)
            {
                reflectors[column] = 0;
                maximumDiagonal = Max(maximumDiagonal, Abs(first));
                continue;
            }

            double transformed = -CopySign(Hypot(first, tailNorm), first);
            double tau = (transformed - first) / transformed;
            double inverse = 1 / (first - transformed);
            q[diagonal] = transformed;
            reflectors[column] = tau;
            maximumDiagonal = Max(maximumDiagonal, Abs(transformed));

            for (int row = column + 1; row < EquationCount; row++)
                q[column * EquationCount + row] *= inverse;

            for (int other = column + 1; other < RightBasisCount; other++)
                ApplyReflector(q, column, tau, q.Slice(other * EquationCount, EquationCount));
        }

        if (!double.IsFinite(maximumDiagonal) || maximumDiagonal == 0)
            return false;

        for (int i = 0; i < RightBasisCount; i++)
        {
            if (Abs(q[i * EquationCount + i]) <= maximumDiagonal * 1e-12)
                return false;
        }

        return true;
    }

    static void BuildPolynomial(
        ReadOnlySpan<double> coefficients,
        ReadOnlySpan<double> reflectors,
        Span<double> polynomial)
    {
        const int eliminatedRows = EquationCount - RightBasisCount;
        Span<double> eliminated = stackalloc double[LeftBasisCount * eliminatedRows * 3];
        Span<double> column = stackalloc double[EquationCount];
        Span<double> trig = stackalloc double[LeftBasisCount * 3];
        Span<double> combination = stackalloc double[LeftBasisCount];
        Span<double> transformed = stackalloc double[LeftBasisCount];
        int matrixLength = PolynomialSize * PolynomialSize;

        for (int part = 0; part < 3; part++)
        {
            int sourceOffset = part * LeftBasisCount * EquationCount;
            int targetOffset = part * LeftBasisCount * eliminatedRows;

            for (int basis = 0; basis < LeftBasisCount; basis++)
            {
                coefficients.Slice(
                    sourceOffset + basis * EquationCount,
                    EquationCount).CopyTo(column);
                var q = coefficients[QOffset..];

                for (int reflector = 0; reflector < RightBasisCount; reflector++)
                    ApplyReflector(q, reflector, reflectors[reflector], column);

                for (int row = 0; row < eliminatedRows; row++)
                {
                    eliminated[targetOffset + basis * eliminatedRows + row] =
                        column[RightBasisCount + row];
                }
            }
        }

        for (int row = 0; row < eliminatedRows; row++)
        {
            for (int basis = 0; basis < LeftBasisCount; basis++)
            {
                trig[basis] = eliminated[basis * eliminatedRows + row];
                trig[LeftBasisCount + basis] = eliminated[
                    LeftBasisCount * eliminatedRows + basis * eliminatedRows + row];
                trig[LeftBasisCount * 2 + basis] = eliminated[
                    LeftBasisCount * eliminatedRows * 2 + basis * eliminatedRows + row];
            }

            for (int part = 0; part < 3; part++)
            {
                for (int basis = 0; basis < LeftBasisCount; basis++)
                {
                    double sine = trig[basis];
                    double cosine = trig[LeftBasisCount + basis];
                    double one = trig[LeftBasisCount * 2 + basis];
                    combination[basis] = part switch
                    {
                        0 => one - cosine,
                        1 => 2 * sine,
                        _ => one + cosine
                    };
                }

                TransformTrig(combination, transformed);
                int matrixOffset = part * matrixLength;

                for (int index = 0; index < LeftBasisCount; index++)
                    polynomial[matrixOffset + row * PolynomialSize + index] = transformed[index];

                for (int index = 0; index < 6; index++)
                    polynomial[matrixOffset + (row + 6) * PolynomialSize + index] = transformed[index + 3];

                for (int index = 0; index < 3; index++)
                    polynomial[matrixOffset + (row + 6) * PolynomialSize + index + 9] = transformed[index];
            }
        }
    }

    static void TransformTrig(ReadOnlySpan<double> value, Span<double> result)
    {
        result[0] = value[3] - value[5] - value[7] + value[8];
        result[1] = 2 * (value[6] - value[2]);
        result[2] = -value[3] - value[5] + value[7] + value[8];
        result[3] = 2 * (value[4] - value[1]);
        result[4] = 4 * value[0];
        result[5] = 2 * (value[1] + value[4]);
        result[6] = -value[3] + value[5] - value[7] + value[8];
        result[7] = 2 * (value[2] + value[6]);
        result[8] = value[3] + value[5] + value[7] + value[8];
    }

    static void SolvePolynomial(
        ReadOnlySpan<Transform> factors,
        Transform target,
        Split split,
        ReadOnlySpan<double> coefficients,
        ReadOnlySpan<double> reflectors,
        ReadOnlySpan<int> permutation,
        ReadOnlySpan<double> polynomial,
        List<double[]> solutions)
    {
        Span<double> pencilA = stackalloc double[PencilSize * PencilSize];
        Span<double> pencilB = stackalloc double[PencilSize * PencilSize];
        BuildPencil(polynomial, pencilA, pencilB);
        Span<double> alphaReal = stackalloc double[PencilSize];
        Span<double> alphaImaginary = stackalloc double[PencilSize];
        Span<double> beta = stackalloc double[PencilSize];

        if (!GeneralizedEigenvalues.TryFind(
            PencilSize,
            pencilA,
            pencilB,
            alphaReal,
            alphaImaginary,
            beta,
            out _))
        {
            return;
        }

        Span<double> angles = stackalloc double[PencilSize];
        int angleCount = 0;
        Span<double> nullVector = stackalloc double[PolynomialSize];
        Span<double> leftBasis = stackalloc double[LeftBasisCount];
        Span<double> rightBasis = stackalloc double[RightBasisCount];
        Span<double> theta = stackalloc double[JointCount];
        var inverseDrop = RigidInverse(factors[split.Drop]);

        for (int root = 0; root < PencilSize; root++)
        {
            double rootScale = Max(Abs(alphaReal[root]), Abs(beta[root]));

            if (!double.IsFinite(rootScale) || rootScale == 0)
                continue;

            if (Abs(alphaImaginary[root]) > 1e-3 * rootScale)
                continue;

            double numerator = alphaReal[root] / rootScale;
            double denominator = beta[root] / rootScale;
            double angle = NormalizeAngle(2 * Atan2(numerator, denominator));
            bool duplicate = false;

            for (int i = 0; i < angleCount; i++)
            {
                if (Abs(NormalizeAngle(angles[i] - angle)) < RootTolerance)
                {
                    duplicate = true;
                    break;
                }
            }

            if (duplicate)
                continue;

            angles[angleCount++] = angle;

            if (!TryNullVector(polynomial, numerator, denominator, nullVector)
                || !TryProjectiveAngle(nullVector, true, out double left0)
                || !TryProjectiveAngle(nullVector, false, out double left1))
            {
                continue;
            }

            FillLeftBasis(left0, left1, leftBasis);

            if (!SolveRight(
                coefficients,
                reflectors,
                permutation,
                angle,
                leftBasis,
                rightBasis))
            {
                continue;
            }

            theta.Clear();
            theta[split.Right0] = Atan2(rightBasis[4], rightBasis[5]);
            theta[split.Right1] = Atan2(rightBasis[6], rightBasis[7]);
            theta[split.Linear] = angle;
            theta[split.Left0] = left0;
            theta[split.Left1] = left1;
            var before = Transform.Identity;

            for (int i = 0; i < split.Drop; i++)
                before *= JointTransform(theta[i], factors[i]);

            var after = Transform.Identity;

            for (int i = split.Drop + 1; i < JointCount; i++)
                after *= JointTransform(theta[i], factors[i]);

            var residual = RigidInverse(before)
                * target
                * RigidInverse(after)
                * inverseDrop;
            theta[split.Drop] = Atan2(
                residual.M10 - residual.M01,
                residual.M00 + residual.M11);
            var actual = before
                * JointTransform(theta[split.Drop], factors[split.Drop])
                * after;

            if (!Closes(actual, target))
                continue;

            if (!Contains(solutions, theta))
                solutions.Add(theta.ToArray());
        }
    }

    static void BuildPencil(
        ReadOnlySpan<double> polynomial,
        Span<double> pencilA,
        Span<double> pencilB)
    {
        pencilA.Clear();
        pencilB.Clear();
        int matrixLength = PolynomialSize * PolynomialSize;
        var quadratic = polynomial[..matrixLength];
        var linear = polynomial.Slice(matrixLength, matrixLength);
        var constant = polynomial.Slice(matrixLength * 2, matrixLength);
        Span<double> rowScale = stackalloc double[PolynomialSize];
        Span<double> columnScale = stackalloc double[PolynomialSize];

        for (int row = 0; row < PolynomialSize; row++)
        {
            double maximum = 0;

            for (int column = 0; column < PolynomialSize; column++)
            {
                int index = row * PolynomialSize + column;
                maximum = Max(maximum, Abs(quadratic[index]));
                maximum = Max(maximum, Abs(linear[index]));
                maximum = Max(maximum, Abs(constant[index]));
            }

            rowScale[row] = maximum == 0 ? 1 : 1 / maximum;
        }

        for (int column = 0; column < PolynomialSize; column++)
        {
            double maximum = 0;

            for (int row = 0; row < PolynomialSize; row++)
            {
                int index = row * PolynomialSize + column;
                maximum = Max(maximum, Abs(quadratic[index] * rowScale[row]));
                maximum = Max(maximum, Abs(linear[index] * rowScale[row]));
                maximum = Max(maximum, Abs(constant[index] * rowScale[row]));
            }

            columnScale[column] = maximum == 0 ? 1 : 1 / maximum;
        }

        for (int i = 0; i < PolynomialSize; i++)
        {
            pencilA[i * PencilSize + i] = 1;
            pencilB[i * PencilSize + i + PolynomialSize] = 1;
        }

        for (int row = 0; row < PolynomialSize; row++)
        {
            for (int column = 0; column < PolynomialSize; column++)
            {
                int source = row * PolynomialSize + column;
                int target = (row + PolynomialSize) * PencilSize + column;
                double scale = rowScale[row] * columnScale[column];
                pencilA[target + PolynomialSize] = constant[source] * scale;
                pencilB[target] = -quadratic[source] * scale;
                pencilB[target + PolynomialSize] = -linear[source] * scale;
            }
        }
    }

    static bool TryNullVector(
        ReadOnlySpan<double> polynomial,
        double numerator,
        double denominator,
        Span<double> result)
    {
        int matrixLength = PolynomialSize * PolynomialSize;
        Span<double> matrix = stackalloc double[matrixLength];
        double quadraticWeight = numerator * numerator;
        double linearWeight = numerator * denominator;
        double constantWeight = denominator * denominator;
        double scale = 0;

        for (int i = 0; i < matrixLength; i++)
        {
            double value = polynomial[i] * quadraticWeight
                + polynomial[matrixLength + i] * linearWeight
                + polynomial[matrixLength * 2 + i] * constantWeight;
            matrix[i] = value;
            scale = Max(scale, Abs(value));
        }

        if (!double.IsFinite(scale) || scale == 0)
            return false;

        for (int i = 0; i < matrixLength; i++)
            matrix[i] /= scale;

        if (TryNullVectorByElimination(matrix, result))
            return true;

        Span<double> vectors = stackalloc double[matrixLength];
        vectors.Clear();

        for (int i = 0; i < PolynomialSize; i++)
            vectors[i * PolynomialSize + i] = 1;

        const int maxSweeps = 50;

        for (int sweep = 0; sweep < maxSweeps; sweep++)
        {
            bool changed = false;

            for (int first = 0; first < PolynomialSize - 1; first++)
            {
                for (int second = first + 1; second < PolynomialSize; second++)
                {
                    double firstNorm = 0;
                    double secondNorm = 0;
                    double dot = 0;

                    for (int row = 0; row < PolynomialSize; row++)
                    {
                        double firstValue = matrix[row * PolynomialSize + first];
                        double secondValue = matrix[row * PolynomialSize + second];
                        firstNorm += firstValue * firstValue;
                        secondNorm += secondValue * secondValue;
                        dot += firstValue * secondValue;
                    }

                    if (firstNorm == 0 || secondNorm == 0)
                        continue;

                    if (Abs(dot) <= 1e-13 * Sqrt(firstNorm * secondNorm))
                        continue;

                    changed = true;
                    double tau = (secondNorm - firstNorm) / (2 * dot);
                    double tangent = CopySign(1 / (Abs(tau) + Hypot(1, tau)), tau);
                    double cosine = 1 / Sqrt(1 + tangent * tangent);
                    double sine = cosine * tangent;
                    RotateColumns(matrix, first, second, cosine, sine);
                    RotateColumns(vectors, first, second, cosine, sine);
                }
            }

            if (!changed)
                break;
        }

        int minimumColumn = 0;
        double minimumNorm = double.MaxValue;
        double maximumNorm = 0;

        for (int column = 0; column < PolynomialSize; column++)
        {
            double norm = 0;

            for (int row = 0; row < PolynomialSize; row++)
            {
                double value = matrix[row * PolynomialSize + column];
                norm += value * value;
            }

            if (norm < minimumNorm)
            {
                minimumNorm = norm;
                minimumColumn = column;
            }

            maximumNorm = Max(maximumNorm, norm);
        }

        if (!double.IsFinite(minimumNorm)
            || maximumNorm == 0
            || Sqrt(minimumNorm / maximumNorm) > 1e-5)
        {
            return false;
        }

        double vectorScale = 0;

        for (int row = 0; row < PolynomialSize; row++)
            vectorScale = Max(vectorScale, Abs(vectors[row * PolynomialSize + minimumColumn]));

        if (vectorScale == 0)
            return false;

        for (int row = 0; row < PolynomialSize; row++)
            result[row] = vectors[row * PolynomialSize + minimumColumn] / vectorScale;

        return true;
    }

    static bool TryNullVectorByElimination(
        ReadOnlySpan<double> matrix,
        Span<double> result)
    {
        Span<double> work = stackalloc double[PolynomialSize * PolynomialSize];
        matrix.CopyTo(work);
        Span<int> permutation = stackalloc int[PolynomialSize];

        for (int i = 0; i < PolynomialSize; i++)
            permutation[i] = i;

        for (int diagonal = 0; diagonal < PolynomialSize - 1; diagonal++)
        {
            int pivotRow = diagonal;
            int pivotColumn = diagonal;
            double pivotMagnitude = 0;

            for (int row = diagonal; row < PolynomialSize; row++)
            {
                for (int column = diagonal; column < PolynomialSize; column++)
                {
                    double magnitude = Abs(work[row * PolynomialSize + column]);

                    if (magnitude > pivotMagnitude)
                    {
                        pivotMagnitude = magnitude;
                        pivotRow = row;
                        pivotColumn = column;
                    }
                }
            }

            if (!double.IsFinite(pivotMagnitude) || pivotMagnitude < 1e-10)
                return false;

            if (pivotRow != diagonal)
            {
                for (int column = 0; column < PolynomialSize; column++)
                {
                    Swap(
                        ref work[diagonal * PolynomialSize + column],
                        ref work[pivotRow * PolynomialSize + column]);
                }
            }

            if (pivotColumn != diagonal)
            {
                for (int row = 0; row < PolynomialSize; row++)
                {
                    Swap(
                        ref work[row * PolynomialSize + diagonal],
                        ref work[row * PolynomialSize + pivotColumn]);
                }

                Swap(ref permutation[diagonal], ref permutation[pivotColumn]);
            }

            double pivot = work[diagonal * PolynomialSize + diagonal];

            for (int row = diagonal + 1; row < PolynomialSize; row++)
            {
                double factor = work[row * PolynomialSize + diagonal] / pivot;
                work[row * PolynomialSize + diagonal] = 0;

                for (int column = diagonal + 1; column < PolynomialSize; column++)
                {
                    work[row * PolynomialSize + column] -=
                        factor * work[diagonal * PolynomialSize + column];
                }
            }
        }

        Span<double> pivoted = stackalloc double[PolynomialSize];
        pivoted[^1] = 1;

        for (int row = PolynomialSize - 2; row >= 0; row--)
        {
            double value = -work[row * PolynomialSize + PolynomialSize - 1];

            for (int column = row + 1; column < PolynomialSize - 1; column++)
                value -= work[row * PolynomialSize + column] * pivoted[column];

            pivoted[row] = value / work[row * PolynomialSize + row];
        }

        double vectorScale = 0;

        for (int i = 0; i < PolynomialSize; i++)
            vectorScale = Max(vectorScale, Abs(pivoted[i]));

        if (!double.IsFinite(vectorScale) || vectorScale == 0)
            return false;

        for (int i = 0; i < PolynomialSize; i++)
            result[permutation[i]] = pivoted[i] / vectorScale;

        double residual = 0;

        for (int row = 0; row < PolynomialSize; row++)
        {
            double value = 0;

            for (int column = 0; column < PolynomialSize; column++)
                value += matrix[row * PolynomialSize + column] * result[column];

            residual = Max(residual, Abs(value));
        }

        return residual <= 1e-5;
    }

    static bool TryProjectiveAngle(
        ReadOnlySpan<double> vector,
        bool first,
        out double angle)
    {
        ReadOnlySpan<(int Numerator, int Denominator)> candidates = first
            ? [(5, 8), (2, 5), (11, 2), (4, 7), (10, 1), (3, 6), (9, 0)]
            : [(7, 8), (6, 7), (1, 2), (4, 5), (10, 11), (3, 4), (0, 1), (9, 10)];
        var selected = candidates[0];
        double magnitude = 0;

        foreach (var candidate in candidates)
        {
            double current = Hypot(
                vector[candidate.Numerator],
                vector[candidate.Denominator]);

            if (current > magnitude)
            {
                magnitude = current;
                selected = candidate;
            }
        }

        if (magnitude < 1e-10)
        {
            angle = 0;
            return false;
        }

        angle = NormalizeAngle(2 * Atan2(
            vector[selected.Numerator],
            vector[selected.Denominator]));
        return true;
    }

    static bool SolveRight(
        ReadOnlySpan<double> coefficients,
        ReadOnlySpan<double> reflectors,
        ReadOnlySpan<int> permutation,
        double linear,
        ReadOnlySpan<double> left,
        Span<double> right)
    {
        Span<double> rhs = stackalloc double[EquationCount];
        var (sine, cosine) = SinCos(linear);

        for (int equation = 0; equation < EquationCount; equation++)
        {
            double value = 0;

            for (int basis = 0; basis < LeftBasisCount; basis++)
            {
                value += (coefficients[PSinOffset + basis * EquationCount + equation] * sine
                    + coefficients[PCosOffset + basis * EquationCount + equation] * cosine
                    + coefficients[POneOffset + basis * EquationCount + equation]) * left[basis];
            }

            rhs[equation] = value;
        }

        var q = coefficients[QOffset..];

        for (int reflector = 0; reflector < RightBasisCount; reflector++)
            ApplyReflector(q, reflector, reflectors[reflector], rhs);

        Span<double> pivoted = stackalloc double[RightBasisCount];

        for (int row = RightBasisCount - 1; row >= 0; row--)
        {
            double value = rhs[row];

            for (int column = row + 1; column < RightBasisCount; column++)
                value -= q[column * EquationCount + row] * pivoted[column];

            double diagonal = q[row * EquationCount + row];

            if (Abs(diagonal) < 1e-14)
                return false;

            pivoted[row] = value / diagonal;
        }

        for (int i = 0; i < RightBasisCount; i++)
            right[permutation[i]] = pivoted[i];

        return Hypot(right[4], right[5]) > 1e-10
            && Hypot(right[6], right[7]) > 1e-10;
    }

    static void ApplyReflector(
        ReadOnlySpan<double> q,
        int column,
        double tau,
        Span<double> vector)
    {
        if (tau == 0)
            return;

        double dot = vector[column];

        for (int row = column + 1; row < EquationCount; row++)
            dot += q[column * EquationCount + row] * vector[row];

        dot *= tau;
        vector[column] -= dot;

        for (int row = column + 1; row < EquationCount; row++)
            vector[row] -= q[column * EquationCount + row] * dot;
    }

    static void FillLeftBasis(double first, double second, Span<double> basis)
    {
        var (sine0, cosine0) = SinCos(first);
        var (sine1, cosine1) = SinCos(second);
        basis[0] = sine0 * sine1;
        basis[1] = sine0 * cosine1;
        basis[2] = cosine0 * sine1;
        basis[3] = cosine0 * cosine1;
        basis[4] = sine0;
        basis[5] = cosine0;
        basis[6] = sine1;
        basis[7] = cosine1;
        basis[8] = 1;
    }

    static void RotateColumns(
        Span<double> matrix,
        int first,
        int second,
        double cosine,
        double sine)
    {
        for (int row = 0; row < PolynomialSize; row++)
        {
            int firstIndex = row * PolynomialSize + first;
            int secondIndex = row * PolynomialSize + second;
            double firstValue = matrix[firstIndex];
            double secondValue = matrix[secondIndex];
            matrix[firstIndex] = cosine * firstValue - sine * secondValue;
            matrix[secondIndex] = sine * firstValue + cosine * secondValue;
        }
    }

    static bool Contains(List<double[]> solutions, ReadOnlySpan<double> candidate)
    {
        foreach (var solution in solutions)
        {
            bool equal = true;

            for (int i = 0; i < JointCount; i++)
            {
                if (Abs(NormalizeAngle(solution[i] - candidate[i])) > RootTolerance)
                {
                    equal = false;
                    break;
                }
            }

            if (equal)
                return true;
        }

        return false;
    }

    static bool Closes(Transform actual, Transform target)
    {
        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 4; column++)
            {
                if (Abs(actual[row, column] - target[row, column]) > 1e-8)
                    return false;
            }
        }

        return true;
    }

    static Transform JointTransform(double theta, Transform factor)
    {
        var (sineTheta, cosineTheta) = SinCos(theta);
        return JointTransform(factor, cosineTheta, sineTheta);
    }

    static Transform JointTransform(
        Transform factor,
        double cosineTheta,
        double sineTheta)
    {
        Transform result = default;
        result.Set(
            cosineTheta * factor.M00 - sineTheta * factor.M10,
            cosineTheta * factor.M01 - sineTheta * factor.M11,
            cosineTheta * factor.M02 - sineTheta * factor.M12,
            cosineTheta * factor.M03 - sineTheta * factor.M13,
            sineTheta * factor.M00 + cosineTheta * factor.M10,
            sineTheta * factor.M01 + cosineTheta * factor.M11,
            sineTheta * factor.M02 + cosineTheta * factor.M12,
            sineTheta * factor.M03 + cosineTheta * factor.M13,
            factor.M20,
            factor.M21,
            factor.M22,
            factor.M23);
        return result;
    }

    static void Swap<T>(ref T first, ref T second) => (second, first) = (first, second);
}
