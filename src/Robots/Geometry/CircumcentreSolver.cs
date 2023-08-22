using Rhino.Geometry;
using static System.Math;

namespace Robots.Geometry;

/// <summary>
/// Code lifted from http://stackoverflow.com/questions/13600739/calculate-centre-of-sphere-whose-surface-contains-4-points-c
/// </summary>
class CircumcentreSolver
{
    double _x, _y, _z;
    readonly double[,] _p = { { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 }, { 0, 0, 0 } };

    internal Point3d Center => new(_x, _y, _z);

    /// <summary>
    /// Computes the center of a sphere such that all four specified points in
    /// 3D space lie on the sphere's surface.
    /// </summary>
    /// <param name="a">The first point (array of 3 doubles for X, Y, Z).</param>
    /// <param name="b">The second point (array of 3 doubles for X, Y, Z).</param>
    /// <param name="c">The third point (array of 3 doubles for X, Y, Z).</param>
    /// <param name="d">The fourth point (array of 3 doubles for X, Y, Z).</param>
    internal CircumcentreSolver(Point3d pa, Point3d pb, Point3d pc, Point3d pd)
    {
        double[] a = { pa.X, pa.Y, pa.Z };
        double[] b = { pb.X, pb.Y, pb.Z };
        double[] c = { pc.X, pc.Y, pc.Z };
        double[] d = { pd.X, pd.Y, pd.Z };
        Compute(a, b, c, d);
    }

    /// <summary>
    /// Evaluate the determinant.
    /// </summary>
    void Compute(double[] a, double[] b, double[] c, double[] d)
    {
        _p[0, 0] = a[0];
        _p[0, 1] = a[1];
        _p[0, 2] = a[2];
        _p[1, 0] = b[0];
        _p[1, 1] = b[1];
        _p[1, 2] = b[2];
        _p[2, 0] = c[0];
        _p[2, 1] = c[1];
        _p[2, 2] = c[2];
        _p[3, 0] = d[0];
        _p[3, 1] = d[1];
        _p[3, 2] = d[2];

        // Compute result sphere.
        Sphere();
    }

    void Sphere()
    {
        double m11, m12, m13, m14;
        double[,] a = { { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 }, { 0, 0, 0, 0 } };

        // Find minor 1, 1.
        for (int i = 0; i < 4; i++)
        {
            a[i, 0] = _p[i, 0];
            a[i, 1] = _p[i, 1];
            a[i, 2] = _p[i, 2];
            a[i, 3] = 1;
        }
        m11 = Determinant(a, 4);

        // Find minor 1, 2.
        for (int i = 0; i < 4; i++)
        {
            a[i, 0] = _p[i, 0] * _p[i, 0] + _p[i, 1] * _p[i, 1] + _p[i, 2] * _p[i, 2];
            a[i, 1] = _p[i, 1];
            a[i, 2] = _p[i, 2];
            a[i, 3] = 1;
        }
        m12 = Determinant(a, 4);

        // Find minor 1, 3.
        for (int i = 0; i < 4; i++)
        {
            a[i, 0] = _p[i, 0] * _p[i, 0] + _p[i, 1] * _p[i, 1] + _p[i, 2] * _p[i, 2];
            a[i, 1] = _p[i, 0];
            a[i, 2] = _p[i, 2];
            a[i, 3] = 1;
        }
        m13 = Determinant(a, 4);

        // Find minor 1, 4.
        for (int i = 0; i < 4; i++)
        {
            a[i, 0] = _p[i, 0] * _p[i, 0] + _p[i, 1] * _p[i, 1] + _p[i, 2] * _p[i, 2];
            a[i, 1] = _p[i, 0];
            a[i, 2] = _p[i, 1];
            a[i, 3] = 1;
        }
        m14 = Determinant(a, 4);

        // Find minor 1, 5.
        for (int i = 0; i < 4; i++)
        {
            a[i, 0] = _p[i, 0] * _p[i, 0] + _p[i, 1] * _p[i, 1] + _p[i, 2] * _p[i, 2];
            a[i, 1] = _p[i, 0];
            a[i, 2] = _p[i, 1];
            a[i, 3] = _p[i, 2];
        }

        // Calculate result.
        if (m11 == 0)
        {
            _x = 0;
            _y = 0;
            _z = 0;
        }
        else
        {
            _x = 0.5 * m12 / m11;
            _y = -0.5 * m13 / m11;
            _z = 0.5 * m14 / m11;
        }
    }

    /// <summary>
    /// Recursive definition of determinate using expansion by minors.
    /// </summary>
    double Determinant(double[,] a, double n)
    {
        int i, j, j1, j2;
        double d;
        double[,] m =
                {
                    { 0, 0, 0, 0 },
                    { 0, 0, 0, 0 },
                    { 0, 0, 0, 0 },
                    { 0, 0, 0, 0 }
                };

        if (n == 2)
        {
            // Terminate recursion.
            d = a[0, 0] * a[1, 1] - a[1, 0] * a[0, 1];
        }
        else
        {
            d = 0;
            for (j1 = 0; j1 < n; j1++) // Do each column.
            {
                for (i = 1; i < n; i++) // Create minor.
                {
                    j2 = 0;
                    for (j = 0; j < n; j++)
                    {
                        if (j == j1) continue;
                        m[i - 1, j2] = a[i, j];
                        j2++;
                    }
                }

                // Sum (+/-)cofactor * minor.
                d += Pow(-1.0, j1) * a[0, j1] * Determinant(m, n - 1);
            }
        }

        return d;
    }
}
