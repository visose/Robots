using static System.Math;
using static Robots.GeometryMath;

namespace Robots;

static class JacobianCondition
{
    const int Size = 6;

    public static double MinimumSingularRatio(Span<double> matrix)
    {
        const int maxSweeps = 20;
        const double orthogonalTolerance = 1e-12;

        if (matrix.Length != Size * Size)
            throw new ArgumentException("A 6 x 6 matrix is required.", nameof(matrix));

        // One-sided Jacobi rotations orthogonalize the columns. Their final
        // norms are the singular values, so no matrix allocation is required.
        for (int sweep = 0; sweep < maxSweeps; sweep++)
        {
            bool changed = false;

            for (int firstCol = 0; firstCol < Size - 1; firstCol++)
            {
                for (int secondCol = firstCol + 1; secondCol < Size; secondCol++)
                {
                    double firstNormSquared = 0;
                    double secondNormSquared = 0;
                    double dot = 0;

                    for (int row = 0; row < Size; row++)
                    {
                        double first = matrix[row * Size + firstCol];
                        double second = matrix[row * Size + secondCol];
                        firstNormSquared += first * first;
                        secondNormSquared += second * second;
                        dot += first * second;
                    }

                    if (firstNormSquared == 0 || secondNormSquared == 0)
                        return 0;

                    if (Abs(dot) <= orthogonalTolerance
                        * Sqrt(firstNormSquared * secondNormSquared))
                    {
                        continue;
                    }

                    changed = true;
                    double tau = (secondNormSquared - firstNormSquared) / (2 * dot);
                    double tangent = CopySign(1 / (Abs(tau) + Hypot(1, tau)), tau);
                    double cosine = 1 / Sqrt(1 + tangent * tangent);
                    double sine = cosine * tangent;

                    for (int row = 0; row < Size; row++)
                    {
                        int firstIndex = row * Size + firstCol;
                        int secondIndex = row * Size + secondCol;
                        double first = matrix[firstIndex];
                        double second = matrix[secondIndex];
                        matrix[firstIndex] = cosine * first - sine * second;
                        matrix[secondIndex] = sine * first + cosine * second;
                    }
                }
            }

            if (!changed)
                break;
        }

        double min = double.MaxValue;
        double max = 0;

        for (int column = 0; column < Size; column++)
        {
            double normSquared = 0;

            for (int row = 0; row < Size; row++)
            {
                double value = matrix[row * Size + column];
                normSquared += value * value;
            }

            min = Min(min, normSquared);
            max = Max(max, normSquared);
        }

        return Sqrt(min / max);
    }

}
