using System.Xml;
using System.Xml.Linq;
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

    // Serialization

    public static XElement GetElement(this XElement element, string name)
    {
        return element.Element(XName.Get(name))
            ??  throw new ArgumentException($" XML tag '{element.Name} is missing '{name} tag.");
    }

    public static XElement? GetElementOrDefault(this XElement element, string name)
    {
        return element.Element(XName.Get(name));
    }

    public static string GetAttribute(this XElement element, string name)
    {
        var attribute = element.Attribute(XName.Get(name))
            ?? throw new ArgumentException($" XML tag '{element.Name} is missing '{name} attribute.");
        
        return attribute.Value;
    }

    public static bool GetBoolAttributeOrDefault(this XElement element, string name)
    {
        string? s = element.Element(XName.Get(name))?.Value;
        return s is not null && XmlConvert.ToBoolean(s);
    }

    public static double GetDoubleAttribute(this XElement element, string name)
    {
        var s = element.GetAttribute(name);
        return XmlConvert.ToDouble(s);
    }

    public static int GetIntAttribute(this XElement element, string name)
    {
        var s = element.GetAttribute(name);
        return XmlConvert.ToInt32(s);
    }

    public static int GetIntAttributeOrDefault(this XElement element, string name)
    {
        string? s = element.Element(XName.Get(name))?.Value;
        return s is null ? 0 : XmlConvert.ToInt32(s);
    }

    // File

    public static string LibraryPath => Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.MyDocuments), "Robots");

    // Collection

    public static T[] Subset<T>(this T[] array, int[] indices)
    {
        T[] subset = new T[indices.Length];
        for (int i = 0; i < indices.Length; i++)
        {
            subset[i] = array[indices[i]];
        }
        return subset;
    }

    public static T[] RangeSubset<T>(this T[] array, int startIndex, int length)
    {
        T[] subset = new T[length];
        Array.Copy(array, startIndex, subset, 0, length);
        return subset;
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

    // Geometry

    public static Transform ToTransform(this double[,] matrix)
    {
        var transform = new Transform();

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                transform[i, j] = matrix[i, j];

        return transform;
    }

    public static Plane ToPlane(this Transform transform)
    {
        Plane plane = Plane.WorldXY;
        plane.Transform(transform);
        return plane;
    }

    public static Transform ToTransform(this Plane plane)
    {
        return Transform.PlaneToPlane(Plane.WorldXY, plane);
    }

    public static double ToRadians(this double value)
    {
        return value * (PI / 180.0);
    }

    public static double ToDegrees(this double value)
    {
        return value * (180.0 / PI);
    }

    // adapted from System.Numerics.Vectors
    public static Quaternion GetRotation(Plane plane)
    {
        var matrix = plane.ToTransform().Transpose();
        double trace = matrix.M00 + matrix.M11 + matrix.M22;

        var q = new Quaternion();

        if (trace > 0.0)
        {
            double s = Sqrt(trace + 1.0);
            q.A = s * 0.5;
            s = 0.5 / s;
            q.B = (matrix.M12 - matrix.M21) * s;
            q.C = (matrix.M20 - matrix.M02) * s;
            q.D = (matrix.M01 - matrix.M10) * s;
        }
        else
        {
            if (matrix.M00 >= matrix.M11 && matrix.M00 >= matrix.M22)
            {
                double s = Sqrt(1.0 + matrix.M00 - matrix.M11 - matrix.M22);
                double invS = 0.5 / s;
                q.B = 0.5 * s;
                q.C = (matrix.M01 + matrix.M10) * invS;
                q.D = (matrix.M02 + matrix.M20) * invS;
                q.A = (matrix.M12 - matrix.M21) * invS;
            }
            else if (matrix.M11 > matrix.M22)
            {
                double s = Sqrt(1.0 + matrix.M11 - matrix.M00 - matrix.M22);
                double invS = 0.5 / s;
                q.B = (matrix.M10 + matrix.M01) * invS;
                q.C = 0.5 * s;
                q.D = (matrix.M21 + matrix.M12) * invS;
                q.A = (matrix.M20 - matrix.M02) * invS;
            }
            else
            {
                double s = Sqrt(1.0 + matrix.M22 - matrix.M00 - matrix.M11);
                double invS = 0.5 / s;
                q.B = (matrix.M20 + matrix.M02) * invS;
                q.C = (matrix.M21 + matrix.M12) * invS;
                q.D = 0.5 * s;
                q.A = (matrix.M01 - matrix.M10) * invS;
            }
        }

        return q;
    }

    public static double Normalize(this Quaternion q, float epsilon = 0)
    {
        double length = q.Scalar;
        if (length > epsilon)
        {
            double invLength = 1.0 / length;
            q.A *= invLength;
            q.B *= invLength;
            q.C *= invLength;
            q.D *= invLength;
        }
        else
        {
            length = 0;
            q.A = q.B = q.C = q.D = 0;
        }
        return length;
    }

    // adapted from System.Numerics.Vectors
    public static Quaternion Slerp(Quaternion q1, Quaternion q2, double t)
    {
        const double epsilon = 1e-6;

        double cosOmega = q1.B * q2.B + q1.C * q2.C +
                         q1.D * q2.D + q1.A * q2.A;

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
            double invSinOmega = 1 / Sin(omega);

            s1 = Sin((1.0 - t) * omega) * invSinOmega;
            s2 = (flip)
                ? -Sin(t * omega) * invSinOmega
                : Sin(t * omega) * invSinOmega;
        }

        Quaternion ans = Quaternion.Identity;

        ans.B = s1 * q1.B + s2 * q2.B;
        ans.C = s1 * q1.C + s2 * q2.C;
        ans.D = s1 * q1.D + s2 * q2.D;
        ans.A = s1 * q1.A + s2 * q2.A;

        return ans;
    }

    public static void GetRotation(this Quaternion q, out Plane plane)
    {
        plane = TransformFromQuaternion(q).ToPlane();
    }

    // adapted from System.Numerics.Vectors
    public static Transform TransformFromQuaternion(Quaternion q)
    {
        var result = new Transform();

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
        result.M01 = 2.0 * (xy + wz);
        result.M02 = 2.0 * (xz - wy);
        result.M03 = 0.0;
        result.M10 = 2.0 * (xy - wz);
        result.M11 = 1.0 - 2.0 * (zz + xx);
        result.M12 = 2.0 * (yz + wx);
        result.M13 = 0.0;
        result.M20 = 2.0 * (xz + wy);
        result.M21 = 2.0 * (yz - wx);
        result.M22 = 1.0 - 2.0 * (yy + xx);
        result.M23 = 0.0;
        result.M30 = 0.0;
        result.M31 = 0.0;
        result.M32 = 0.0;
        result.M33 = 1.0;

        result = result.Transpose();

        return result;
    }
}
