using static System.Math;

namespace Robots;

/// <summary>
/// Finds the generalized eigenvalues of a real matrix pencil using the QZ algorithm.
/// </summary>
/// <remarks>
/// This is an eigenvalue-only, zero-based translation of the public-domain Netlib
/// EISPACK routines QZHES, QZIT, and QZVAL. Matrices are row-major and are overwritten.
/// Sources: https://www.netlib.org/eispack/qzhes.f,
/// https://www.netlib.org/eispack/qzit.f, and https://www.netlib.org/eispack/qzval.f.
/// Each returned eigenvalue is represented homogeneously as
/// <c>(alphaReal + i alphaImaginary) / beta</c>; a zero beta represents an infinite
/// eigenvalue. The pencil must be finite and regular, and buffers must not overlap.
/// The caller should equilibrate badly scaled pencils.
/// </remarks>
static class GeneralizedEigenvalues
{
    public const int MaximumOrder = 24;

    // Binary64 unit roundoff. double.Epsilon is the least positive subnormal value.
    const double UnitRoundoff = 2.2204460492503131e-16;

    /// <summary>
    /// Finds the generalized eigenvalues of <paramref name="matrixA"/> - lambda
    /// <paramref name="matrixB"/> without allocating working storage.
    /// </summary>
    /// <param name="order">Matrix order, from 1 through <see cref="MaximumOrder"/>.</param>
    /// <param name="matrixA">Caller-owned row-major matrix A; overwritten.</param>
    /// <param name="matrixB">Caller-owned row-major matrix B; overwritten.</param>
    /// <param name="alphaReal">Real homogeneous numerators.</param>
    /// <param name="alphaImaginary">Imaginary homogeneous numerators.</param>
    /// <param name="beta">Non-negative homogeneous denominators.</param>
    /// <param name="unconvergedIndex">
    /// Zero-based eigenvalue index at which QZ iteration stopped, or -1 on success.
    /// </param>
    /// <returns>
    /// True when every eigenvalue converged. Eigenvalue outputs are unspecified on failure.
    /// </returns>
    public static bool TryFind(
        int order,
        Span<double> matrixA,
        Span<double> matrixB,
        Span<double> alphaReal,
        Span<double> alphaImaginary,
        Span<double> beta,
        out int unconvergedIndex)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(order, 1);
        ArgumentOutOfRangeException.ThrowIfGreaterThan(order, MaximumOrder);

        int matrixLength = order * order;
        ArgumentOutOfRangeException.ThrowIfLessThan(matrixA.Length, matrixLength, nameof(matrixA));
        ArgumentOutOfRangeException.ThrowIfLessThan(matrixB.Length, matrixLength, nameof(matrixB));
        ArgumentOutOfRangeException.ThrowIfLessThan(alphaReal.Length, order, nameof(alphaReal));
        ArgumentOutOfRangeException.ThrowIfLessThan(alphaImaginary.Length, order, nameof(alphaImaginary));
        ArgumentOutOfRangeException.ThrowIfLessThan(beta.Length, order, nameof(beta));

        ReduceToHessenbergTriangular(order, matrixA, matrixB);

        if (!ReduceToQuasiTriangular(order, matrixA, matrixB, out double epsilonB, out unconvergedIndex))
            return false;

        ExtractEigenvalues(order, matrixA, matrixB, epsilonB, alphaReal, alphaImaginary, beta);
        return true;
    }

    /// <summary>
    /// EISPACK QZHES with MATZ=false: reduces A to upper Hessenberg form and B
    /// to upper triangular form by orthogonal equivalence transformations.
    /// </summary>
    static void ReduceToHessenbergTriangular(int order, Span<double> a, Span<double> b)
    {
        if (order <= 1)
            return;

        int last = order - 1;

        for (int l = 0; l < last; l++)
        {
            int next = l + 1;
            double scale = 0;

            for (int i = next; i < order; i++)
                scale += Abs(b[i * order + l]);

            if (scale == 0)
                continue;

            scale += Abs(b[l * order + l]);
            double norm = 0;

            for (int i = l; i < order; i++)
            {
                int index = i * order + l;
                b[index] /= scale;
                norm += b[index] * b[index];
            }

            double reflector = SignedMagnitude(Sqrt(norm), b[l * order + l]);
            b[l * order + l] += reflector;
            double rho = reflector * b[l * order + l];

            for (int j = next; j < order; j++)
            {
                double product = 0;

                for (int i = l; i < order; i++)
                    product += b[i * order + l] * b[i * order + j];

                product = -product / rho;

                for (int i = l; i < order; i++)
                    b[i * order + j] += product * b[i * order + l];
            }

            for (int j = 0; j < order; j++)
            {
                double product = 0;

                for (int i = l; i < order; i++)
                    product += b[i * order + l] * a[i * order + j];

                product = -product / rho;

                for (int i = l; i < order; i++)
                    a[i * order + j] += product * b[i * order + l];
            }

            b[l * order + l] = -scale * reflector;

            for (int i = next; i < order; i++)
                b[i * order + l] = 0;
        }

        if (order == 2)
            return;

        for (int k = 0; k < order - 2; k++)
        {
            for (int l = order - 2; l > k; l--)
            {
                int next = l + 1;
                double scale = Abs(a[l * order + k]) + Abs(a[next * order + k]);

                if (scale == 0)
                    continue;

                double u1 = a[l * order + k] / scale;
                double u2 = a[next * order + k] / scale;
                double reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2), u1);
                double v1 = -(u1 + reflector) / reflector;
                double v2 = -u2 / reflector;
                u2 = v2 / v1;

                for (int j = k; j < order; j++)
                {
                    double product = a[l * order + j] + u2 * a[next * order + j];
                    a[l * order + j] += product * v1;
                    a[next * order + j] += product * v2;
                }

                a[next * order + k] = 0;

                for (int j = l; j < order; j++)
                {
                    double product = b[l * order + j] + u2 * b[next * order + j];
                    b[l * order + j] += product * v1;
                    b[next * order + j] += product * v2;
                }

                scale = Abs(b[next * order + next]) + Abs(b[next * order + l]);

                if (scale == 0)
                    continue;

                u1 = b[next * order + next] / scale;
                u2 = b[next * order + l] / scale;
                reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2), u1);
                v1 = -(u1 + reflector) / reflector;
                v2 = -u2 / reflector;
                u2 = v2 / v1;

                for (int i = 0; i <= next; i++)
                {
                    double product = b[i * order + next] + u2 * b[i * order + l];
                    b[i * order + next] += product * v1;
                    b[i * order + l] += product * v2;
                }

                b[next * order + l] = 0;

                for (int i = 0; i < order; i++)
                {
                    double product = a[i * order + next] + u2 * a[i * order + l];
                    a[i * order + next] += product * v1;
                    a[i * order + l] += product * v2;
                }
            }
        }
    }

    /// <summary>
    /// EISPACK QZIT with EPS1=0 and MATZ=false: reduces the Hessenberg-triangular
    /// pair to quasi-triangular-triangular form.
    /// </summary>
    static bool ReduceToQuasiTriangular(
        int order,
        Span<double> a,
        Span<double> b,
        out double epsilonB,
        out int unconvergedIndex)
    {
        int l;
        int l1;
        int lm1;
        int lowerBound;
        int previous;
        int previous2;
        int shiftType;
        double a1;
        double a2;
        double a3 = 0;
        double a11;
        double a21;
        double shift = 0;

        double normA = 0;
        double normB = 0;

        for (int i = 0; i < order; i++)
        {
            double rowA = i == 0 ? 0 : Abs(a[i * order + i - 1]);
            double rowB = 0;

            for (int j = i; j < order; j++)
            {
                rowA += Abs(a[i * order + j]);
                rowB += Abs(b[i * order + j]);
            }

            normA = Max(normA, rowA);
            normB = Max(normB, rowB);
        }

        if (normA == 0)
            normA = 1;

        if (normB == 0)
            normB = 1;

        double epsilonA = UnitRoundoff * normA;
        epsilonB = UnitRoundoff * normB;
        int last = order - 1;
        int remainingIterations = 30 * order;
        int lastColumn;
        int iterations;

    BeginQzStep:
        if (last <= 1)
            goto Success;

        lastColumn = last;
        iterations = 0;
        previous = last - 1;
        previous2 = previous - 1;

    CheckConvergence:
        shiftType = 2;
        l = last;

        while (l > 0 && Abs(a[l * order + l - 1]) > epsilonA)
            l--;

        if (l > 0)
        {
            lm1 = l - 1;
            a[l * order + lm1] = 0;

            if (l >= previous)
            {
                last = lm1;
                goto BeginQzStep;
            }
        }

    SetBlockStart:
        lowerBound = l;

    CheckSmallB:
        l1 = l + 1;
        double b11 = b[l * order + l];

        if (Abs(b11) <= epsilonB)
        {
            b[l * order + l] = 0;
            double scale = Abs(a[l * order + l]) + Abs(a[l1 * order + l]);
            double u1 = a[l * order + l] / scale;
            double u2 = a[l1 * order + l] / scale;
            double reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2), u1);
            double v1 = -(u1 + reflector) / reflector;
            double v2 = -u2 / reflector;
            u2 = v2 / v1;

            for (int j = l; j <= lastColumn; j++)
            {
                double product = a[l * order + j] + u2 * a[l1 * order + j];
                a[l * order + j] += product * v1;
                a[l1 * order + j] += product * v2;
                product = b[l * order + j] + u2 * b[l1 * order + j];
                b[l * order + j] += product * v1;
                b[l1 * order + j] += product * v2;
            }

            if (l != 0)
                a[l * order + l - 1] = -a[l * order + l - 1];

            lm1 = l;
            l = l1;
            a[l * order + lm1] = 0;

            if (l < previous)
                goto SetBlockStart;

            last = lm1;
            goto BeginQzStep;
        }

        a11 = a[l * order + l] / b11;
        a21 = a[l1 * order + l] / b11;

        if (shiftType == 1)
            goto FormSingleShift;

        if (remainingIterations == 0)
        {
            unconvergedIndex = last;
            return false;
        }

        if (iterations == 10)
            goto FormAdHocShift;

        double b22 = b[l1 * order + l1];

        if (Abs(b22) < epsilonB)
            b22 = epsilonB;

        double b33 = b[previous * order + previous];

        if (Abs(b33) < epsilonB)
            b33 = epsilonB;

        double b44 = b[last * order + last];

        if (Abs(b44) < epsilonB)
            b44 = epsilonB;

        double a33 = a[previous * order + previous] / b33;
        double a34 = a[previous * order + last] / b44;
        double a43 = a[last * order + previous] / b33;
        double a44 = a[last * order + last] / b44;
        double b34 = b[previous * order + last] / b44;
        double halfTrace = 0.5 * (a43 * b34 - a33 - a44);
        double discriminant = halfTrace * halfTrace + a34 * a43 - a33 * a44;

        if (discriminant < 0)
            goto FormDoubleShift;

        shiftType = 1;
        double root = Sqrt(discriminant);
        shift = -halfTrace + root;
        double alternative = -halfTrace - root;

        if (Abs(alternative - a44) < Abs(shift - a44))
            shift = alternative;

        for (int search = lowerBound; search <= previous2; search++)
        {
            l = previous2 + lowerBound - search;

            if (l == lowerBound)
                goto FormSingleShift;

            lm1 = l - 1;
            l1 = l + 1;
            double test = a[l * order + l];

            if (Abs(b[l * order + l]) > epsilonB)
                test -= shift * b[l * order + l];

            if (Abs(a[l * order + lm1]) <= Abs(test / a[l1 * order + l]) * epsilonA)
                goto CheckSmallB;
        }

    FormSingleShift:
        a1 = a11 - shift;
        a2 = a21;

        if (l != lowerBound)
            a[l * order + l - 1] = -a[l * order + l - 1];

        goto ApplyShift;

    FormDoubleShift:
        double a12 = a[l * order + l1] / b22;
        double a22 = a[l1 * order + l1] / b22;
        double b12 = b[l * order + l1] / b22;
        a1 = ((a33 - a11) * (a44 - a11) - a34 * a43 + a43 * b34 * a11)
            / a21 + a12 - a11 * b12;
        a2 = a22 - a11 - a21 * b12 - (a33 - a11) - (a44 - a11) + a43 * b34;
        a3 = a[(l1 + 1) * order + l1] / b22;
        goto ApplyShift;

    FormAdHocShift:
        a1 = 0;
        a2 = 1;
        a3 = 1.1605;

    ApplyShift:
        iterations++;
        remainingIterations--;

        for (int k = l; k <= previous; k++)
        {
            bool threeElement = k != previous && shiftType == 2;
            int k1 = k + 1;
            int k2 = k + 2;
            int km1 = Max(k - 1, l);
            int rowEnd = Min(last, k1 + shiftType);

            if (!threeElement)
            {
                if (k != l)
                {
                    a1 = a[k * order + km1];
                    a2 = a[k1 * order + km1];
                }

                double scale = Abs(a1) + Abs(a2);

                if (scale == 0)
                    goto CheckConvergence;

                double u1 = a1 / scale;
                double u2 = a2 / scale;
                double reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2), u1);
                double v1 = -(u1 + reflector) / reflector;
                double v2 = -u2 / reflector;
                u2 = v2 / v1;

                for (int j = km1; j <= lastColumn; j++)
                {
                    double product = a[k * order + j] + u2 * a[k1 * order + j];
                    a[k * order + j] += product * v1;
                    a[k1 * order + j] += product * v2;
                    product = b[k * order + j] + u2 * b[k1 * order + j];
                    b[k * order + j] += product * v1;
                    b[k1 * order + j] += product * v2;
                }

                if (k != l)
                    a[k1 * order + km1] = 0;
            }
            else
            {
                if (k != l)
                {
                    a1 = a[k * order + km1];
                    a2 = a[k1 * order + km1];
                    a3 = a[k2 * order + km1];
                }

                double scale = Abs(a1) + Abs(a2) + Abs(a3);

                if (scale == 0)
                    continue;

                double u1 = a1 / scale;
                double u2 = a2 / scale;
                double u3 = a3 / scale;
                double reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2 + u3 * u3), u1);
                double v1 = -(u1 + reflector) / reflector;
                double v2 = -u2 / reflector;
                double v3 = -u3 / reflector;
                u2 = v2 / v1;
                u3 = v3 / v1;

                for (int j = km1; j <= lastColumn; j++)
                {
                    double product = a[k * order + j] + u2 * a[k1 * order + j] + u3 * a[k2 * order + j];
                    a[k * order + j] += product * v1;
                    a[k1 * order + j] += product * v2;
                    a[k2 * order + j] += product * v3;
                    product = b[k * order + j] + u2 * b[k1 * order + j] + u3 * b[k2 * order + j];
                    b[k * order + j] += product * v1;
                    b[k1 * order + j] += product * v2;
                    b[k2 * order + j] += product * v3;
                }

                if (k != l)
                {
                    a[k1 * order + km1] = 0;
                    a[k2 * order + km1] = 0;
                }

                scale = Abs(b[k2 * order + k2]) + Abs(b[k2 * order + k1]) + Abs(b[k2 * order + k]);

                if (scale != 0)
                {
                    u1 = b[k2 * order + k2] / scale;
                    u2 = b[k2 * order + k1] / scale;
                    u3 = b[k2 * order + k] / scale;
                    reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2 + u3 * u3), u1);
                    v1 = -(u1 + reflector) / reflector;
                    v2 = -u2 / reflector;
                    v3 = -u3 / reflector;
                    u2 = v2 / v1;
                    u3 = v3 / v1;

                    for (int i = lowerBound; i <= rowEnd; i++)
                    {
                        double product = a[i * order + k2] + u2 * a[i * order + k1] + u3 * a[i * order + k];
                        a[i * order + k2] += product * v1;
                        a[i * order + k1] += product * v2;
                        a[i * order + k] += product * v3;
                        product = b[i * order + k2] + u2 * b[i * order + k1] + u3 * b[i * order + k];
                        b[i * order + k2] += product * v1;
                        b[i * order + k1] += product * v2;
                        b[i * order + k] += product * v3;
                    }

                    b[k2 * order + k] = 0;
                    b[k2 * order + k1] = 0;
                }
            }

            double pairScale = Abs(b[k1 * order + k1]) + Abs(b[k1 * order + k]);

            if (pairScale == 0)
                continue;

            double pairU1 = b[k1 * order + k1] / pairScale;
            double pairU2 = b[k1 * order + k] / pairScale;
            double pairReflector = SignedMagnitude(Sqrt(pairU1 * pairU1 + pairU2 * pairU2), pairU1);
            double pairV1 = -(pairU1 + pairReflector) / pairReflector;
            double pairV2 = -pairU2 / pairReflector;
            pairU2 = pairV2 / pairV1;

            for (int i = lowerBound; i <= rowEnd; i++)
            {
                double product = a[i * order + k1] + pairU2 * a[i * order + k];
                a[i * order + k1] += product * pairV1;
                a[i * order + k] += product * pairV2;
                product = b[i * order + k1] + pairU2 * b[i * order + k];
                b[i * order + k1] += product * pairV1;
                b[i * order + k] += product * pairV2;
            }

            b[k1 * order + k] = 0;
        }

        goto CheckConvergence;

    Success:
        unconvergedIndex = -1;
        return true;
    }

    /// <summary>
    /// EISPACK QZVAL with MATZ=false: extracts homogeneous eigenvalues from the
    /// quasi-triangular-triangular pair.
    /// </summary>
    static void ExtractEigenvalues(
        int order,
        Span<double> a,
        Span<double> b,
        double epsilonB,
        Span<double> alphaReal,
        Span<double> alphaImaginary,
        Span<double> beta)
    {
        bool skipConjugate = false;

        for (int en = order - 1; en >= 0; en--)
        {
            if (skipConjugate)
            {
                skipConjugate = false;
                continue;
            }

            int previous = en - 1;

            if (en == 0 || a[en * order + previous] == 0)
            {
                double real = a[en * order + en];

                if (b[en * order + en] < 0)
                    real = -real;

                alphaReal[en] = real;
                alphaImaginary[en] = 0;
                beta[en] = Abs(b[en * order + en]);
                continue;
            }

            double an = 0;
            double bn = 0;
            double eigenvalueEstimate = 0;
            double a1;
            double a2;

            if (Abs(b[previous * order + previous]) <= epsilonB)
            {
                a1 = a[previous * order + previous];
                a2 = a[en * order + previous];
                ApplyRealLeftTransformation(order, previous, en, a1, a2, a, b);
                StoreRealPair(order, previous, en, a, b, alphaReal, alphaImaginary, beta);
                skipConjugate = true;
                continue;
            }

            if (Abs(b[en * order + en]) <= epsilonB)
            {
                a1 = a[en * order + en];
                a2 = a[en * order + previous];
            }
            else
            {
                an = Abs(a[previous * order + previous])
                    + Abs(a[previous * order + en])
                    + Abs(a[en * order + previous])
                    + Abs(a[en * order + en]);
                bn = Abs(b[previous * order + previous])
                    + Abs(b[previous * order + en])
                    + Abs(b[en * order + en]);
                double a11 = a[previous * order + previous] / an;
                double a12 = a[previous * order + en] / an;
                double a21 = a[en * order + previous] / an;
                double a22 = a[en * order + en] / an;
                double b11 = b[previous * order + previous] / bn;
                double b12 = b[previous * order + en] / bn;
                double b22 = b[en * order + en] / bn;
                double e = a11 / b11;
                double alternate = a22 / b22;
                double s = a21 / (b11 * b22);
                double t = (a22 - e * b22) / b22;

                if (Abs(e) > Abs(alternate))
                {
                    e = alternate;
                    t = (a11 - e * b11) / b11;
                }

                double c = 0.5 * (t - s * b12);
                double discriminant = c * c + s * (a12 - e * b12);

                if (discriminant < 0)
                {
                    StoreComplexPair(
                        previous,
                        en,
                        an,
                        bn,
                        e,
                        c,
                        discriminant,
                        a11,
                        a12,
                        a21,
                        a22,
                        b11,
                        b12,
                        b22,
                        alphaReal,
                        alphaImaginary,
                        beta);
                    skipConjugate = true;
                    continue;
                }

                eigenvalueEstimate = e + c + SignedMagnitude(Sqrt(discriminant), c);
                a11 -= eigenvalueEstimate * b11;
                a12 -= eigenvalueEstimate * b12;
                a22 -= eigenvalueEstimate * b22;

                if (Abs(a11) + Abs(a12) >= Abs(a21) + Abs(a22))
                {
                    a1 = a12;
                    a2 = a11;
                }
                else
                {
                    a1 = a22;
                    a2 = a21;
                }
            }

            ApplyRealRightTransformation(order, previous, en, a1, a2, a, b);

            if (bn != 0)
            {
                if (an >= Abs(eigenvalueEstimate) * bn)
                {
                    a1 = b[previous * order + previous];
                    a2 = b[en * order + previous];
                }
                else
                {
                    a1 = a[previous * order + previous];
                    a2 = a[en * order + previous];
                }

                ApplyRealLeftTransformation(order, previous, en, a1, a2, a, b);
            }

            StoreRealPair(order, previous, en, a, b, alphaReal, alphaImaginary, beta);
            skipConjugate = true;
        }
    }

    static void ApplyRealRightTransformation(
        int order,
        int previous,
        int current,
        double a1,
        double a2,
        Span<double> a,
        Span<double> b)
    {
        double scale = Abs(a1) + Abs(a2);
        double u1 = a1 / scale;
        double u2 = a2 / scale;
        double reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2), u1);
        double v1 = -(u1 + reflector) / reflector;
        double v2 = -u2 / reflector;
        u2 = v2 / v1;

        for (int i = 0; i <= current; i++)
        {
            double product = a[i * order + current] + u2 * a[i * order + previous];
            a[i * order + current] += product * v1;
            a[i * order + previous] += product * v2;
            product = b[i * order + current] + u2 * b[i * order + previous];
            b[i * order + current] += product * v1;
            b[i * order + previous] += product * v2;
        }
    }

    static void ApplyRealLeftTransformation(
        int order,
        int previous,
        int current,
        double a1,
        double a2,
        Span<double> a,
        Span<double> b)
    {
        double scale = Abs(a1) + Abs(a2);

        if (scale == 0)
            return;

        double u1 = a1 / scale;
        double u2 = a2 / scale;
        double reflector = SignedMagnitude(Sqrt(u1 * u1 + u2 * u2), u1);
        double v1 = -(u1 + reflector) / reflector;
        double v2 = -u2 / reflector;
        u2 = v2 / v1;

        for (int j = previous; j < order; j++)
        {
            double product = a[previous * order + j] + u2 * a[current * order + j];
            a[previous * order + j] += product * v1;
            a[current * order + j] += product * v2;
            product = b[previous * order + j] + u2 * b[current * order + j];
            b[previous * order + j] += product * v1;
            b[current * order + j] += product * v2;
        }
    }

    static void StoreRealPair(
        int order,
        int previous,
        int current,
        Span<double> a,
        Span<double> b,
        Span<double> alphaReal,
        Span<double> alphaImaginary,
        Span<double> beta)
    {
        a[current * order + previous] = 0;
        b[current * order + previous] = 0;
        double first = a[previous * order + previous];
        double second = a[current * order + current];

        if (b[previous * order + previous] < 0)
            first = -first;

        if (b[current * order + current] < 0)
            second = -second;

        alphaReal[previous] = first;
        alphaReal[current] = second;
        alphaImaginary[previous] = 0;
        alphaImaginary[current] = 0;
        beta[previous] = Abs(b[previous * order + previous]);
        beta[current] = Abs(b[current * order + current]);
    }

    static void StoreComplexPair(
        int previous,
        int current,
        double an,
        double bn,
        double e,
        double c,
        double discriminant,
        double a11,
        double a12,
        double a21,
        double a22,
        double b11,
        double b12,
        double b22,
        Span<double> alphaReal,
        Span<double> alphaImaginary,
        Span<double> beta)
    {
        e += c;
        double ei = Sqrt(-discriminant);
        double a11Real = a11 - e * b11;
        double a11Imaginary = ei * b11;
        double a12Real = a12 - e * b12;
        double a12Imaginary = ei * b12;
        double a22Real = a22 - e * b22;
        double a22Imaginary = ei * b22;
        double z1Real;
        double z1Imaginary;
        double z2Real;
        double z2Imaginary;

        if (Abs(a11Real) + Abs(a11Imaginary) + Abs(a12Real) + Abs(a12Imaginary)
            >= Abs(a21) + Abs(a22Real) + Abs(a22Imaginary))
        {
            z1Real = a12Real;
            z1Imaginary = a12Imaginary;
            z2Real = -a11Real;
            z2Imaginary = -a11Imaginary;
        }
        else
        {
            z1Real = a22Real;
            z1Imaginary = a22Imaginary;
            z2Real = -a21;
            z2Imaginary = 0;
        }

        double cosineZ = Sqrt(z1Real * z1Real + z1Imaginary * z1Imaginary);
        double sineZReal;
        double sineZImaginary;

        if (cosineZ == 0)
        {
            sineZReal = 1;
            sineZImaginary = 0;
        }
        else
        {
            sineZReal = (z1Real * z2Real + z1Imaginary * z2Imaginary) / cosineZ;
            sineZImaginary = (z1Real * z2Imaginary - z1Imaginary * z2Real) / cosineZ;
            double norm = Sqrt(cosineZ * cosineZ + sineZReal * sineZReal + sineZImaginary * sineZImaginary);
            cosineZ /= norm;
            sineZReal /= norm;
            sineZImaginary /= norm;
        }

        double q1Real;
        double q1Imaginary;
        double q2Real;
        double q2Imaginary;

        if (an >= (Abs(e) + ei) * bn)
        {
            q1Real = cosineZ * b11 + sineZReal * b12;
            q1Imaginary = sineZImaginary * b12;
            q2Real = sineZReal * b22;
            q2Imaginary = sineZImaginary * b22;
        }
        else
        {
            q1Real = cosineZ * a11 + sineZReal * a12;
            q1Imaginary = sineZImaginary * a12;
            q2Real = cosineZ * a21 + sineZReal * a22;
            q2Imaginary = sineZImaginary * a22;
        }

        double cosineQ = Sqrt(q1Real * q1Real + q1Imaginary * q1Imaginary);
        double sineQReal;
        double sineQImaginary;

        if (cosineQ == 0)
        {
            sineQReal = 1;
            sineQImaginary = 0;
        }
        else
        {
            sineQReal = (q1Real * q2Real + q1Imaginary * q2Imaginary) / cosineQ;
            sineQImaginary = (q1Real * q2Imaginary - q1Imaginary * q2Real) / cosineQ;
            double norm = Sqrt(cosineQ * cosineQ + sineQReal * sineQReal + sineQImaginary * sineQImaginary);
            cosineQ /= norm;
            sineQReal /= norm;
            sineQImaginary /= norm;
        }

        double ssReal = sineQReal * sineZReal + sineQImaginary * sineZImaginary;
        double ssImaginary = sineQReal * sineZImaginary - sineQImaginary * sineZReal;

        for (int pass = 0; pass < 2; pass++)
        {
            double tr;
            double ti;
            double dr;
            double di;

            if (pass == 0)
            {
                tr = cosineQ * cosineZ * a11
                    + cosineQ * sineZReal * a12
                    + sineQReal * cosineZ * a21
                    + ssReal * a22;
                ti = cosineQ * sineZImaginary * a12
                    - sineQImaginary * cosineZ * a21
                    + ssImaginary * a22;
                dr = cosineQ * cosineZ * b11
                    + cosineQ * sineZReal * b12
                    + ssReal * b22;
                di = cosineQ * sineZImaginary * b12 + ssImaginary * b22;
            }
            else
            {
                tr = ssReal * a11
                    - sineQReal * cosineZ * a12
                    - cosineQ * sineZReal * a21
                    + cosineQ * cosineZ * a22;
                ti = -ssImaginary * a11
                    - sineQImaginary * cosineZ * a12
                    + cosineQ * sineZImaginary * a21;
                dr = ssReal * b11
                    - sineQReal * cosineZ * b12
                    + cosineQ * cosineZ * b22;
                di = -ssImaginary * b11 - sineQImaginary * cosineZ * b12;
            }

            double imaginaryNumerator = ti * dr - tr * di;
            int index = imaginaryNumerator < 0 ? current : previous;
            double denominator = Sqrt(dr * dr + di * di);
            beta[index] = bn * denominator;
            alphaReal[index] = an * (tr * dr + ti * di) / denominator;
            alphaImaginary[index] = an * imaginaryNumerator / denominator;
        }
    }

    static double SignedMagnitude(double magnitude, double sign) => sign < 0 ? -Abs(magnitude) : Abs(magnitude);
}
