using System.Reflection;
using Rhino.Geometry;
using static System.Math;

namespace Robots;

static class Util
{
    // Constants

    public const double DistanceTol = 0.001;
    public const double AngleTol = 0.001;
    public const double TimeTol = 0.00001;
    public const double UnitTol = 0.000001;
    public const double SingularityTol = 0.0001;
    public const double HalfPI = PI * 0.5;
    public const double PI2 = PI * 2.0;

    // Exceptions

    public static T NotNull<T>(this T? value, string? text = null)
    {
        return value ?? throw new ArgumentNullException(nameof(value), text);
    }

    // Resources

    public static StreamReader GetResource(string name)
    {
        var assembly = Assembly.GetExecutingAssembly();
        var resourceName = $"Robots.Resources.{name}";
        var stream = assembly.GetManifestResourceStream(resourceName);
        return new StreamReader(stream);
    }

    public static string GetStringResource(string name)
    {
        using var reader = GetResource(name);
        return reader.ReadToEnd();
    }

    // String

    public static bool EqualsIgnoreCase(this string? a, string? b)
    {
        return string.Equals(a, b, StringComparison.OrdinalIgnoreCase);
    }

    // Collection

    public static IList<T> TryCastIList<T>(this IEnumerable<T> list)
    {
        return list as IList<T> ?? list.ToList();
    }

    public static T[] TryCastArray<T>(this IEnumerable<T> list)
    {
        return list as T[] ?? list.ToArray();
    }

    public static List<K> MapToList<T, K>(this IList<T> array, Func<T, K> projection)
    {
        var result = new List<K>(array.Count);

        for (int i = 0; i < array.Count; i++)
            result.Add(projection(array[i]));

        return result;
    }

    public static K[] Map<T, K>(this IList<T> array, Func<T, K> projection)
    {
        var result = new K[array.Count];

        for (int i = 0; i < array.Count; i++)
            result[i] = projection(array[i]);

        return result;
    }

    public static K[] Map<T, K>(this IList<T> array, Func<T, int, K> projection)
    {
        var result = new K[array.Count];

        for (int i = 0; i < array.Count; i++)
            result[i] = projection(array[i], i);

        return result;
    }

    public static T[] Subset<T>(this T[] array, int[] indices)
    {
        T[] subset = new T[indices.Length];

        for (int i = 0; i < indices.Length; i++)
            subset[i] = array[indices[i]];

        return subset;
    }

    public static T[] RangeSubset<T>(this T[] array, int startIndex, int length)
    {
        T[] subset = new T[length];
        Array.Copy(array, startIndex, subset, 0, length);
        return subset;
    }

    public static T MaxBy<T, K>(this IEnumerable<T> list, Func<T, K> comparable) where K : IComparable<K>
    {
        if (!list.Any())
            throw new ArgumentOutOfRangeException(nameof(list), "List cannot be empty.");

        T maxItem = list.First();
        K maxValue = comparable(maxItem);

        foreach (var item in list.Skip(1))
        {
            var val = comparable(item);

            if (val.CompareTo(maxValue) > 0)
            {
                maxItem = item;
                maxValue = val;
            }
        }

        return maxItem;
    }

    public static IEnumerable<List<T>> Transpose<T>(this IEnumerable<IEnumerable<T>> source)
    {
        var enumerators = source.Select(e => e.GetEnumerator()).ToArray();
        try
        {
            while (enumerators.All(e => e.MoveNext()))
            {
                yield return enumerators.Select(e => e.Current).ToList();
            }
        }
        finally
        {
            foreach (var enumerator in enumerators)
                enumerator.Dispose();
        }
    }

    public static byte[] Combine(params byte[][] arrays)
    {
        var result = new byte[arrays.Sum(x => x.Length)];
        int offset = 0;

        foreach (byte[] data in arrays)
        {
            Buffer.BlockCopy(data, 0, result, offset, data.Length);
            offset += data.Length;
        }

        return result;
    }

    // Geometry

    public static double Sqr(double n) => n * n;
    public static double ToRadians(this double n) => n * (PI / 180.0);
    public static double ToDegrees(this double n) => n * (180.0 / PI);
    public static double ToMeters(this double n) => n * 0.001;
    public static Point3d ToMeters(this Point3d n) => n * 0.001;
    public static double FromMeters(this double n) => n * 1000.0;

    // Transform

    public static Vector3d GetColumn3d(this ref Transform t, int col)
    {
        return new(t[0, col], t[1, col], t[2, col]);
    }

    public static void Set(this ref Transform t, double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
    {
        t.M00 = m00; t.M01 = m01; t.M02 = m02; t.M03 = m03;
        t.M10 = m10; t.M11 = m11; t.M12 = m12; t.M13 = m13;
        t.M20 = m20; t.M21 = m21; t.M22 = m22; t.M23 = m23;
        t.M33 = 1;
    }

    public static void SetRotation(this ref Transform t, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
    {
        t.M00 = m00; t.M01 = m01; t.M02 = m02;
        t.M10 = m10; t.M11 = m11; t.M12 = m12;
        t.M20 = m20; t.M21 = m21; t.M22 = m22;
        t.M33 = 1;
    }

    public static Plane ToPlane(this ref Transform t)
    {
        var p = new Point3d(t.M03, t.M13, t.M23);
        var vx = new Vector3d(t.M00, t.M10, t.M20);
        var vy = new Vector3d(t.M01, t.M11, t.M21);

        var vz = Vector3d.CrossProduct(vx, vy);
        vy = Vector3d.CrossProduct(vz, vx);
        vx.Normalize();
        vy.Normalize();
        vz.Normalize();

        Plane result = default;
        result.Origin = p;
        result.XAxis = vx;
        result.YAxis = vy;
        result.ZAxis = vz;

        return result;
    }

    public static Vector6d ToEulerZYX(this ref Transform t)
    {
        double a = Atan2(-t.M10, t.M00);
        double mult = 1.0 - t.M20 * t.M20;
        if (Abs(mult) < UnitTol) mult = 0.0;
        double b = Atan2(t.M20, Sqrt(mult));
        double c = Atan2(-t.M21, t.M22);

        if (t.M20 < (-1.0 + UnitTol))
        {
            a = Atan2(t.M01, t.M11);
            b = -PI / 2;
            c = 0;
        }
        else if (t.M20 > (1.0 - UnitTol))
        {
            a = Atan2(t.M01, t.M11);
            b = PI / 2;
            c = 0;
        }

        return new(t.M03, t.M13, t.M23, -a, -b, -c);
    }

    public static Transform EulerZYXToTransform(this Vector6d euler)
    {
        double a = -euler.A4;
        double b = -euler.A5;
        double c = -euler.A6;
        double ca = Cos(a);
        double sa = Sin(a);
        double cb = Cos(b);
        double sb = Sin(b);
        double cc = Cos(c);
        double sc = Sin(c);

        Transform t = default;
        t.M00 = ca * cb; t.M01 = sa * cc + ca * sb * sc; t.M02 = sa * sc - ca * sb * cc;
        t.M10 = -sa * cb; t.M11 = ca * cc - sa * sb * sc; t.M12 = ca * sc + sa * sb * cc;
        t.M20 = sb; t.M21 = -cb * sc; t.M22 = cb * cc;
        t.M33 = 1;
        t.M03 = euler.A1; t.M13 = euler.A2; t.M23 = euler.A3;

        return t;
    }

    // Vector3d

    public static void Normalize(this ref Vector3d v)
    {
        double x = v.X;
        double y = v.Y;
        double z = v.Z;
        double lengthSq = x * x + y * y + z * z;
        double length = Sqrt(lengthSq);
        v.X = x / length;
        v.Y = y / length;
        v.Z = z / length;
    }

    // Plane

    public static void InverseOrient(this ref Plane a, ref Plane b)
    {
        a.Transform(b.ToInverseTransform());
    }

    public static void Orient(this ref Plane a, ref Plane b)
    {
        a.Transform(b.ToTransform());
    }

    public static Transform ToTransform(this ref Plane plane)
    {
        Transform t = default;
        var vx = plane.XAxis;
        var vy = plane.YAxis;
        var vz = plane.ZAxis;
        t.M00 = vx.X;
        t.M01 = vy.X;
        t.M02 = vz.X;
        t.M03 = plane.OriginX;
        t.M10 = vx.Y;
        t.M11 = vy.Y;
        t.M12 = vz.Y;
        t.M13 = plane.OriginY;
        t.M20 = vx.Z;
        t.M21 = vy.Z;
        t.M22 = vz.Z;
        t.M23 = plane.OriginZ;
        t.M33 = 1;
        return t;
    }

    public static Transform ToInverseTransform(this ref Plane plane)
    {
        Transform t = default;
        var vx = plane.XAxis;
        var vy = plane.YAxis;
        var vz = plane.ZAxis;
        var p = -(Vector3d)plane.Origin;
        t.M00 = vx.X;
        t.M01 = vx.Y;
        t.M02 = vx.Z;
        t.M03 = p * vx;
        t.M10 = vy.X;
        t.M11 = vy.Y;
        t.M12 = vy.Z;
        t.M13 = p * vy;
        t.M20 = vz.X;
        t.M21 = vz.Y;
        t.M22 = vz.Z;
        t.M23 = p * vz;
        t.M33 = 1;
        return t;
    }

    // adapted from System.Numerics.Vectors
    public static Quaternion ToQuaternion(this ref Plane plane)
    {
        var matrix = plane.ToTransform();
        double trace = matrix.M00 + matrix.M11 + matrix.M22;

        Quaternion q = default;

        if (trace > 0.0)
        {
            double s = Sqrt(trace + 1.0);
            q.A = s * 0.5;
            s = 0.5 / s;
            q.B = (matrix.M21 - matrix.M12) * s;
            q.C = (matrix.M02 - matrix.M20) * s;
            q.D = (matrix.M10 - matrix.M01) * s;
        }
        else
        {
            if (matrix.M00 >= matrix.M11 && matrix.M00 >= matrix.M22)
            {
                double s = Sqrt(1.0 + matrix.M00 - matrix.M11 - matrix.M22);
                double invS = 0.5 / s;
                q.B = 0.5 * s;
                q.C = (matrix.M10 + matrix.M01) * invS;
                q.D = (matrix.M20 + matrix.M02) * invS;
                q.A = (matrix.M21 - matrix.M12) * invS;
            }
            else if (matrix.M11 > matrix.M22)
            {
                double s = Sqrt(1.0 + matrix.M11 - matrix.M00 - matrix.M22);
                double invS = 0.5 / s;
                q.B = (matrix.M01 + matrix.M10) * invS;
                q.C = 0.5 * s;
                q.D = (matrix.M12 + matrix.M21) * invS;
                q.A = (matrix.M02 - matrix.M20) * invS;
            }
            else
            {
                double s = Sqrt(1.0 + matrix.M22 - matrix.M00 - matrix.M11);
                double invS = 0.5 / s;
                q.B = (matrix.M02 + matrix.M20) * invS;
                q.C = (matrix.M12 + matrix.M21) * invS;
                q.D = 0.5 * s;
                q.A = (matrix.M10 - matrix.M01) * invS;
            }
        }

        return q;
    }

    // Quaternion

    // adapted from System.Numerics.Vectors
    public static Quaternion Slerp(ref Quaternion q1, ref Quaternion q2, double t)
    {
        const double epsilon = 1e-6;

        double cosOmega = q1.B * q2.B + q1.C * q2.C + q1.D * q2.D + q1.A * q2.A;

        bool flip = false;

        if (cosOmega < 0.0)
        {
            flip = true;
            cosOmega = -cosOmega;
        }

        double s1, s2;

        if (cosOmega > (1.0 - epsilon))
        {
            // Too close, do straight linear interpolation.
            s1 = 1.0 - t;
            s2 = (flip) ? -t : t;
        }
        else
        {
            double omega = Acos(cosOmega);
            double invSinOmega = 1.0 / Sin(omega);

            s1 = Sin((1.0 - t) * omega) * invSinOmega;
            s2 = (flip)
                ? -Sin(t * omega) * invSinOmega
                : Sin(t * omega) * invSinOmega;
        }

        Quaternion ans = default;

        ans.B = s1 * q1.B + s2 * q2.B;
        ans.C = s1 * q1.C + s2 * q2.C;
        ans.D = s1 * q1.D + s2 * q2.D;
        ans.A = s1 * q1.A + s2 * q2.A;

        return ans;
    }

    public static Plane ToPlane(this ref Quaternion quaternion, Point3d point)
    {
        quaternion.GetRotation(out Plane plane);
        plane.Origin = point;
        return plane;
    }

    // adapted from System.Numerics.Vectors
    public static Transform ToTransform(this ref Quaternion q)
    {
        Transform result = default;

        double xx = q.B * q.B;
        double yy = q.C * q.C;
        double zz = q.D * q.D;

        double xy = q.B * q.C;
        double wz = q.D * q.A;
        double xz = q.D * q.B;
        double wy = q.C * q.A;
        double yz = q.C * q.D;
        double wx = q.B * q.A;

        result.M00 = 1.0 - 2.0 * (yy + zz);
        result.M01 = 2.0 * (xy - wz);
        result.M02 = 2.0 * (xz + wy);
        //result.M03 = 0.0;
        result.M10 = 2.0 * (xy + wz);
        result.M11 = 1.0 - 2.0 * (zz + xx);
        result.M12 = 2.0 * (yz - wx);
        //result.M13 = 0.0;
        result.M20 = 2.0 * (xz - wy);
        result.M21 = 2.0 * (yz + wx);
        result.M22 = 1.0 - 2.0 * (yy + xx);
        //result.M23 = 0.0;
        //result.M30 = 0.0;
        //result.M31 = 0.0;
        //result.M32 = 0.0;
        result.M33 = 1.0;

        return result;
    }

    // Matrix

    public static double[,] Mult(this double[,] a, double[,] b)
    {
        int rA = a.GetLength(0);
        int cA = a.GetLength(1);
        int rB = b.GetLength(0);
        int cB = b.GetLength(1);

        if (cA != rB)
            throw new("Matrices have incompatible dimensions.");

        var result = new double[rA, cB];

        for (int i = 0; i < rA; i++)
        {
            for (int j = 0; j < cB; j++)
            {
                double n = 0;

                for (int k = 0; k < cA; k++)
                    n += a[i, k] * b[k, j];

                result[i, j] = n;
            }
        }

        return result;
    }

    public static double[] Mult(this double[] a, double[,] b)
    {
        int cA = a.Length;
        int rB = b.GetLength(0);
        int cB = b.GetLength(1);

        if (cA != rB)
            throw new("Matrices have incompatible dimensions.");

        var result = new double[cB];

        for (int j = 0; j < cB; j++)
        {
            double n = 0;

            for (int k = 0; k < cA; k++)
                n += a[k] * b[k, j];

            result[j] = n;
        }

        return result;
    }

    public static double[,] Transpose(this double[,] matrix)
    {
        int w = matrix.GetLength(0);
        int h = matrix.GetLength(1);

        var result = new double[h, w];

        for (int i = 0; i < w; i++)
        {
            for (int j = 0; j < h; j++)
                result[j, i] = matrix[i, j];
        }

        return result;
    }
}
