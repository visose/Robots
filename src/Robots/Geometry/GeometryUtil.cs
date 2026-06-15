using static System.Math;

using Rhino.Geometry;

using static Robots.Util;

namespace Robots;

public static class GeometryUtil
{
    internal static double Clamp(double value, double min, double max) => Min(Max(value, min), max);

    extension(double n)
    {
        internal double ToRadians() => n * (PI / 180.0);

        internal double ToDegrees() => n * (180.0 / PI);

        internal double ToMeters() => n * 0.001;

        internal double FromMeters() => n * 1000.0;
    }

    extension(Point3d n)
    {
        internal Point3d ToMeters() => n * 0.001;
    }

    /// <summary>
    /// Quaternion interpolation based on: http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
    /// </summary>
    public static Plane QuaternionLerp(Plane a, Plane b, double t, double min, double max)
    {
        t = NormalizeLerpParameter(t, min, max);
        var origin = a.Origin * (1 - t) + b.Origin * t;

        var qa = a.ToQuaternion();
        var qb = b.ToQuaternion();
        var q = Slerp(ref qa, ref qb, t);

        return q.ToPlane(origin);
    }

    public static Plane MatrixLerp(Plane a, Plane b, double t, double min, double max)
    {
        t = NormalizeLerpParameter(t, min, max);

        var ta = a.ToTransform();
        var tb = b.ToTransform();
        var result = Transform.Identity;

        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
                result[i, j] = ta[i, j] * (1.0 - t) + tb[i, j] * t;
        }

        return result.ToPlane();
    }

    internal static Plane CheckPlane(Plane plane, string name) =>
        plane.IsValid ? plane : throw new ArgumentException("Plane is invalid.", name);

    public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
    {
        var point = new Point3d(CheckFinite(x, nameof(x)), CheckFinite(y, nameof(y)), CheckFinite(z, nameof(z)));
        var quaternion = new Quaternion(CheckFinite(q1, nameof(q1)), CheckFinite(q2, nameof(q2)), CheckFinite(q3, nameof(q3)), CheckFinite(q4, nameof(q4)));
        return quaternion.ToPlane(point);
    }

    public static Plane QuaternionToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 7);

        var point = new Point3d(numbers[0], numbers[1], numbers[2]);
        var quaternion = new Quaternion(numbers[3], numbers[4], numbers[5], numbers[6]);
        return quaternion.ToPlane(point);
    }

    public static double[] PlaneToQuaternion(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var q = plane.ToQuaternion();
        return [plane.OriginX, plane.OriginY, plane.OriginZ, q.A, q.B, q.C, q.D];
    }

    internal static double[] PlaneToEulerZYXDegrees(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return [e.A1, e.A2, e.A3, e.A4.ToDegrees(), e.A5.ToDegrees(), e.A6.ToDegrees()];
    }

    internal static double[] PlaneToEulerZYZDegrees(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var t = plane.ToTransform();
        var e = t.ToEulerZYZ();
        return [e.A1, e.A2, e.A3, e.A4.ToDegrees(), e.A5.ToDegrees(), e.A6.ToDegrees()];
    }

    internal static Vector6d PlaneToEulerXYZ(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var t = plane.ToTransform();
        return t.ToEulerXYZ();
    }

    internal static double[] PlaneToReversedEulerZYXDegrees(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return [e.A1, e.A2, e.A3, e.A6.ToDegrees(), e.A5.ToDegrees(), e.A4.ToDegrees()];
    }

    internal static Plane EulerZYXDegreesToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 6);
        var euler = new Vector6d(numbers[0], numbers[1], numbers[2], numbers[3].ToRadians(), numbers[4].ToRadians(), numbers[5].ToRadians());
        var t = euler.EulerZYXToTransform();
        return t.ToPlane();
    }

    internal static Plane EulerZYZDegreesToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 6);
        var euler = new Vector6d(numbers[0], numbers[1], numbers[2], numbers[3].ToRadians(), numbers[4].ToRadians(), numbers[5].ToRadians());
        var t = euler.EulerZYZToTransform();
        return t.ToPlane();
    }

    internal static Plane ReversedEulerZYXDegreesToPlane(double[] numbers)
    {
        numbers = CheckNumbers(numbers, 6);
        var euler = new Vector6d(numbers[0], numbers[1], numbers[2], numbers[5].ToRadians(), numbers[4].ToRadians(), numbers[3].ToRadians());
        var t = euler.EulerZYXToTransform();
        return t.ToPlane();
    }

    internal static Plane EulerXYZToPlane(Vector6d euler)
    {
        euler = CheckFiniteEuler(euler, nameof(euler));
        var t = euler.EulerXYZToTransform();
        return t.ToPlane();
    }

    /// <summary>
    /// Code lifted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
    /// </summary>
    internal static double[] PlaneToAxisAngle(Plane plane)
    {
        plane = CheckPlane(plane, nameof(plane));
        var t = plane.ToTransform();

        const double epsilon = 0.01;
        const double epsilon2 = 0.1;

        if (IsAxisAngleSingularity(ref t, epsilon))
        {
            if (IsIdentityRotation(ref t, epsilon2))
                return AxisAngleNumbers(plane, default);

            var vector = AxisForPiRotation(ref t, epsilon);
            vector *= PI;
            return AxisAngleNumbers(plane, vector);
        }

        return AxisAngleNumbers(plane, AxisAngleVector(ref t));
    }

    internal static Plane AxisAngleToPlane(double x, double y, double z, double vx, double vy, double vz)
    {
        x = CheckFinite(x, nameof(x));
        y = CheckFinite(y, nameof(y));
        z = CheckFinite(z, nameof(z));
        vx = CheckFinite(vx, nameof(vx));
        vy = CheckFinite(vy, nameof(vy));
        vz = CheckFinite(vz, nameof(vz));

        var matrix = AxisAngleRotation(vx, vy, vz);
        var plane = matrix.ToPlane();
        plane.Origin = new Point3d(x, y, z);
        return plane;
    }

    static bool IsAxisAngleSingularity(ref Transform t, double epsilon)
    {
        return Abs(t.M01 - t.M10) < epsilon
            && Abs(t.M02 - t.M20) < epsilon
            && Abs(t.M12 - t.M21) < epsilon;
    }

    static bool IsIdentityRotation(ref Transform t, double epsilon)
    {
        return Abs(t.M01 + t.M10) < epsilon
            && Abs(t.M02 + t.M20) < epsilon
            && Abs(t.M12 + t.M21) < epsilon
            && Abs(t.M00 + t.M11 + t.M22 - 3) < epsilon;
    }

    static Vector3d AxisForPiRotation(ref Transform t, double epsilon)
    {
        double xx = (t.M00 + 1) / 2;
        double yy = (t.M11 + 1) / 2;
        double zz = (t.M22 + 1) / 2;
        double xy = (t.M01 + t.M10) / 4;
        double xz = (t.M02 + t.M20) / 4;
        double yz = (t.M12 + t.M21) / 4;

        Vector3d vector;

        if (xx > yy && xx > zz)
        {
            if (xx < epsilon)
            {
                vector = new(0, 0.7071, 0.7071);
            }
            else
            {
                double x = Sqrt(xx);
                vector = new(x, xy / x, xz / x);
            }
        }
        else if (yy > zz)
        {
            if (yy < epsilon)
            {
                vector = new(0.7071, 0, 0.7071);
            }
            else
            {
                double y = Sqrt(yy);
                vector = new(xy / y, y, yz / y);
            }
        }
        else
        {
            if (zz < epsilon)
            {
                vector = new(0.7071, 0.7071, 0);
            }
            else
            {
                double z = Sqrt(zz);
                vector = new(xz / z, yz / z, z);
            }
        }

        _ = vector.Unitize();
        return vector;
    }

    static Vector3d AxisAngleVector(ref Transform t)
    {
        double s = Sqrt((t.M21 - t.M12) * (t.M21 - t.M12)
          + (t.M02 - t.M20) * (t.M02 - t.M20)
          + (t.M10 - t.M01) * (t.M10 - t.M01));

        if (Abs(s) < 0.001)
            s = 1;

        double angle = Acos((t.M00 + t.M11 + t.M22 - 1) / 2);
        var vector = new Vector3d((t.M21 - t.M12) / s, (t.M02 - t.M20) / s, (t.M10 - t.M01) / s);
        _ = vector.Unitize();
        vector *= angle;
        return vector;
    }

    static double[] AxisAngleNumbers(Plane plane, Vector3d vector)
    {
        return [plane.OriginX, plane.OriginY, plane.OriginZ, vector.X, vector.Y, vector.Z];
    }

    static Transform AxisAngleRotation(double vx, double vy, double vz)
    {
        var vector = new Vector3d(vx, vy, vz);
        double angle = vector.Length;
        _ = vector.Unitize();

        double c = Cos(angle);
        double s = Sin(angle);
        double t = 1.0 - c;

        double xx = vector.X * vector.X * t;
        double yy = vector.Y * vector.Y * t;
        double zz = vector.Z * vector.Z * t;
        double xy = vector.X * vector.Y * t;
        double xz = vector.X * vector.Z * t;
        double yz = vector.Y * vector.Z * t;
        double xs = vector.X * s;
        double ys = vector.Y * s;
        double zs = vector.Z * s;

        var matrix = Transform.Identity;
        matrix.SetRotation(
            c + xx, xy - zs, xz + ys,
            xy + zs, c + yy, yz - xs,
            xz - ys, yz + xs, c + zz);
        return matrix;
    }

    extension(ref Transform t)
    {
        internal Vector3d GetColumn3d(int col)
        {
            return new(t[0, col], t[1, col], t[2, col]);
        }

        internal void Set(double m00, double m01, double m02, double m03, double m10, double m11, double m12, double m13, double m20, double m21, double m22, double m23)
        {
            t.M00 = m00; t.M01 = m01; t.M02 = m02; t.M03 = m03;
            t.M10 = m10; t.M11 = m11; t.M12 = m12; t.M13 = m13;
            t.M20 = m20; t.M21 = m21; t.M22 = m22; t.M23 = m23;
            t.M33 = 1;
        }

        internal void SetRotation(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
        {
            t.M00 = m00; t.M01 = m01; t.M02 = m02;
            t.M10 = m10; t.M11 = m11; t.M12 = m12;
            t.M20 = m20; t.M21 = m21; t.M22 = m22;
            t.M33 = 1;
        }

        internal Plane ToPlane()
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

        internal Vector6d ToEulerZYX()
        {
            double a = Atan2(-t.M10, t.M00);
            double mult = 1.0 - t.M20 * t.M20;

            if (Abs(mult) < UnitTol)
                mult = 0.0;

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

        internal Vector6d ToEulerZYZ()
        {
            double alpha, beta, gamma;

            const double epsilon = 1E-12;

            if (Abs(t.M22) > 1 - epsilon)
            {
                gamma = 0.0;

                if (t.M22 > 0)
                {
                    beta = 0.0;
                    alpha = Atan2(t.M10, t.M00);
                }
                else
                {
                    beta = PI;
                    alpha = Atan2(-t.M10, -t.M00);
                }
            }
            else
            {
                alpha = Atan2(t.M12, t.M02);
                beta = Atan2(Sqrt(t.M20 * t.M20 + t.M21 * t.M21), t.M22);
                gamma = Atan2(t.M21, -t.M20);
            }

            return new(t.M03, t.M13, t.M23, alpha, beta, gamma);
        }

        internal Vector6d ToEulerXYZ()
        {
            double a = Atan2(-t.M12, t.M22);
            double mult = 1.0 - t.M02 * t.M02;

            if (Abs(mult) < UnitTol)
                mult = 0.0;

            double b = Atan2(t.M02, Sqrt(mult));
            double c = Atan2(-t.M01, t.M00);

            if (t.M02 < (-1.0 + UnitTol))
            {
                a = Atan2(t.M21, t.M11);
                b = -PI / 2;
                c = 0;
            }
            else if (t.M02 > (1.0 - UnitTol))
            {
                a = Atan2(t.M21, t.M11);
                b = PI / 2;
                c = 0;
            }

            return new(t.M03, t.M13, t.M23, a, b, c);
        }
    }

    internal static Transform RotationZ(double angle)
    {
        double cos = Cos(angle);
        double sin = Sin(angle);
        Transform t = default;
        t.SetRotation(cos, -sin, 0, sin, cos, 0, 0, 0, 1);
        return t;
    }

    extension(Vector6d euler)
    {
        internal Transform EulerZYXToTransform()
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
            t.Set(
                ca * cb, sa * cc + ca * sb * sc, sa * sc - ca * sb * cc, euler.A1,
                -sa * cb, ca * cc - sa * sb * sc, ca * sc + sa * sb * cc, euler.A2,
                sb, -cb * sc, cb * cc, euler.A3);

            return t;
        }

        internal Transform EulerXYZToTransform()
        {
            double a = euler.A4;
            double b = euler.A5;
            double c = euler.A6;
            double ca = Cos(a);
            double sa = Sin(a);
            double cb = Cos(b);
            double sb = Sin(b);
            double cc = Cos(c);
            double sc = Sin(c);

            Transform t = default;
            t.SetRotation(
                cb * cc, ca * sc + sa * sb * cc, sa * sc - ca * sb * cc,
                -cb * sc, ca * cc - sa * sb * sc, sa * cc + ca * sb * sc,
                sb, -sa * cb, ca * cb);
            t.M03 = euler.A1;
            t.M13 = euler.A2;
            t.M23 = euler.A3;
            return t;
        }

        internal Transform EulerZYZToTransform()
        {
            double a = euler.A4;
            double b = euler.A5;
            double c = euler.A6;
            double ca = Cos(a), sa = Sin(a), cb = Cos(b), sb = Sin(b), cc = Cos(c), sc = Sin(c);

            Transform t = default;
            t.Set(
                ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb, euler.A1,
                sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb, euler.A2,
                -sb * cc, sb * sc, cb, euler.A3);
            return t;
        }
    }

    extension(ref Vector3d v)
    {
        internal void Normalize()
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
    }

    extension(ref Plane a)
    {
        internal void InverseOrient(ref Plane b)
        {
            _ = a.Transform(b.ToInverseTransform());
        }

        internal void Orient(ref Plane b)
        {
            _ = a.Transform(b.ToTransform());
        }

        internal Transform PlaneToPlane(ref Plane to)
        {
            return to.ToTransform() * a.ToInverseTransform();
        }

        internal Transform ToTransform()
        {
            Transform t = default;
            var vx = a.XAxis;
            var vy = a.YAxis;
            var vz = a.ZAxis;
            t.M00 = vx.X;
            t.M01 = vy.X;
            t.M02 = vz.X;
            t.M03 = a.OriginX;
            t.M10 = vx.Y;
            t.M11 = vy.Y;
            t.M12 = vz.Y;
            t.M13 = a.OriginY;
            t.M20 = vx.Z;
            t.M21 = vy.Z;
            t.M22 = vz.Z;
            t.M23 = a.OriginZ;
            t.M33 = 1;
            return t;
        }

        internal Transform ToInverseTransform()
        {
            Transform t = default;
            var vx = a.XAxis;
            var vy = a.YAxis;
            var vz = a.ZAxis;
            var p = -(Vector3d)a.Origin;
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
        internal Quaternion ToQuaternion()
        {
            var matrix = a.ToTransform();
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
    }

    // adapted from System.Numerics.Vectors
    internal static Quaternion Slerp(ref Quaternion q1, ref Quaternion q2, double t)
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
            s2 = flip ? -t : t;
        }
        else
        {
            double omega = Acos(cosOmega);
            double invSinOmega = 1.0 / Sin(omega);

            s1 = Sin((1.0 - t) * omega) * invSinOmega;
            s2 = flip
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

    extension(ref Quaternion quaternion)
    {
        internal Plane ToPlane(Point3d point)
        {
            var transform = quaternion.ToTransform();
            var plane = transform.ToPlane();
            plane.Origin = point;
            return plane;
        }

        // adapted from System.Numerics.Vectors
        internal Transform ToTransform()
        {
            Transform result = default;

            double xx = quaternion.B * quaternion.B;
            double yy = quaternion.C * quaternion.C;
            double zz = quaternion.D * quaternion.D;

            double xy = quaternion.B * quaternion.C;
            double wz = quaternion.D * quaternion.A;
            double xz = quaternion.D * quaternion.B;
            double wy = quaternion.C * quaternion.A;
            double yz = quaternion.C * quaternion.D;
            double wx = quaternion.B * quaternion.A;

            result.SetRotation(
                1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy),
                2.0 * (xy + wz), 1.0 - 2.0 * (zz + xx), 2.0 * (yz - wx),
                2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (yy + xx));

            return result;
        }
    }

    static double NormalizeLerpParameter(double t, double min, double max)
    {
        t = (t - min) / (max - min);
        return double.IsNaN(t) ? 0 : t;
    }

    static double[] CheckNumbers(double[] numbers, int length)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(numbers.Length, length, nameof(numbers));
        return CheckFinite(numbers, nameof(numbers), "Numbers must be finite.");
    }

    static Vector6d CheckFiniteEuler(Vector6d vector, string name)
    {
        for (int i = 0; i < 6; i++)
            _ = CheckFinite(vector[i], name, "Euler values must be finite.");

        return vector;
    }
}
