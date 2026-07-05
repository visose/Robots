using System.Diagnostics.CodeAnalysis;
using static System.Math;
using Rhino.Geometry;

namespace Robots;

class NumericalKinematics(RobotArm robot, bool useModifiedDH = false, int? redundant = null) : RobotKinematics(robot)
{
    readonly int _jointCount = robot.Joints.Length;
    readonly bool _useModifiedDH = useModifiedDH;
    readonly int? _redundant = redundant;
    readonly double[] _midJoints = robot.Joints.Map(j => j.Range.Mid);

    protected override int SolutionCount => 1;
    internal override bool RequiresContinuation => true;
    internal override int? RedundantJointIndex => _redundant;

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        if (_useModifiedDH)
            return ModifiedDH(joints);

        return DH(joints);
    }

    protected override double[] InverseKinematics(Transform t, RobotConfigurations configuration, double[] external, PreviousJoints prevJoints, out List<string> errors)
    {
        const double min = 1e-5;
        const double max = 0.3;

        errors = [];
        var previous = prevJoints.HasValue ? prevJoints.Values : _midJoints.AsSpan();
        var joints = previous.ToArray();

        if (_redundant is int redundant)
            joints[redundant] = external.Length > 0 ? external[0] : previous[redundant];

        for (int i = 0; i < 400; i++)
        {
            Transform forward = default;
            double[,]? inverse = null;

            for (int ii = 0; ii < 20; ii++)
            {
                forward = Forward(joints);
                var jacobian = Jacobian(joints, ref forward);

                if (TryPseudoInverse(jacobian, out var candidate))
                {
                    inverse = candidate;
                    break;
                }

                for (int j = 0; j < joints.Length; j++)
                    joints[j] += 1e-3;
            }

            if (inverse is null)
            {
                errors.Add("Target near singularity.");
                return joints;
            }

            var transpose = inverse.Transpose();
            var jd = Subtract(ref forward, ref t);
            var deltas = jd.Mult(transpose);
            var maxValue = deltas.Max(Abs);

            if (maxValue > max)
            {
                var value = maxValue / max;

                for (int j = 0; j < deltas.Length; j++)
                    deltas[j] /= value;
            }

            for (int j = 0; j < deltas.Length; j++)
                joints[j] += j == _redundant ? 0 : deltas[j];

            if (maxValue <= min)
                return joints;
        }

        errors.Add("Target out of reach.");
        return joints;
    }

    Transform Forward(double[] joints)
    {
        return ForwardKinematics(joints)[joints.Length - 1];
    }

    double[,] Jacobian(double[] joints, ref Transform f)
    {
        const double step = 0.001;
        var m = new double[6, _jointCount];
        var move = new double[joints.Length];
        Array.Copy(joints, move, joints.Length);

        for (int i = 0; i < joints.Length; i++)
        {
            if (i == _redundant)
                continue;

            move[i] += step;
            Transform d = Forward(move);
            move[i] = joints[i];
            var step2 = step * 2;

            m[0, i] = (d[0, 3] - f[0, 3]) / step;
            m[1, i] = (d[1, 3] - f[1, 3]) / step;
            m[2, i] = (d[2, 3] - f[2, 3]) / step;
            m[3, i] = (d[2, 0] * f[1, 0] + d[2, 1] * f[1, 1] + f[1, 2] * d[2, 2] - d[1, 0] * f[2, 0] - f[2, 1] * d[1, 1] - d[1, 2] * f[2, 2]) / step2;
            m[4, i] = (f[2, 0] * d[0, 0] + d[0, 1] * f[2, 1] + d[0, 2] * f[2, 2] - d[2, 0] * f[0, 0] - d[2, 1] * f[0, 1] - f[0, 2] * d[2, 2]) / step2;
            m[5, i] = (d[1, 0] * f[0, 0] + f[0, 1] * d[1, 1] + d[1, 2] * f[0, 2] - f[1, 0] * d[0, 0] - d[0, 1] * f[1, 1] - d[0, 2] * f[1, 2]) / step2;
        }

        return m;
    }

    static double[] Subtract(ref Transform f, ref Transform t)
    {
        const double tol = 5e-07;

        Vector6d v = default;
        v[0] = +t[2, 0] * f[1, 0] + t[2, 1] * f[1, 1] + f[1, 2] * t[2, 2];
        v[1] = +t[0, 1] * f[2, 1] + t[0, 2] * f[2, 2] + f[2, 0] * t[0, 0];
        v[2] = +t[1, 0] * f[0, 0] + t[1, 2] * f[0, 2] + f[0, 1] * t[1, 1];
        v[3] = -t[1, 0] * f[2, 0] - t[1, 2] * f[2, 2] - f[2, 1] * t[1, 1];
        v[4] = -t[2, 0] * f[0, 0] - t[2, 1] * f[0, 1] - f[0, 2] * t[2, 2];
        v[5] = -t[0, 1] * f[1, 1] - t[0, 2] * f[1, 2] - f[1, 0] * t[0, 0];

        var wrap = true;

        for (int i = 0; i < 3; i++)
        {
            if (Abs(Abs(v[i]) - Abs(v[i + 3])) > tol)
                wrap = false;
        }

        var delta = new double[6];

        for (int i = 0; i < 3; i++)
        {
            if (wrap && ((v[i] > 0 && v[i + 3] < 0) || (v[i] < 0 && v[i + 3] > 0)))
                v[i + 3] *= -1;

            delta[i] = t[i, 3] - f[i, 3];
            delta[i + 3] = (v[i] + v[i + 3]) * 0.5;
        }

        return delta;
    }

    static bool TryPseudoInverse(double[,] jacobian, [MaybeNullWhen(false)] out double[,] result)
    {
        const double tol = 1e-6;

        var rows = jacobian.GetLength(0);
        var columns = jacobian.GetLength(1);

        var reverse = rows < columns;
        var transpose = jacobian.Transpose();
        var matrix = reverse ? jacobian.Mult(transpose) : transpose.Mult(jacobian);

        int length = Min(rows, columns);
        int length2 = length * 2;

        var values = new double[length * length * 2];

        // Copy to flat array and set diagonal to 1
        for (int i = 0, j = 0; i < length; i++, j += length2)
        {
            for (int k = 0; k < length; k++)
                values[j + k] = matrix[i, k];

            values[j + i + length] = 1;
        }

        // Gaussian elimination transform into row echelon form
        for (int i = 0, j = 0; i < length - 1; i++, j += length2)
        {
            for (int k = i + 1; k < length; k++)
            {
                int stride = k * length2;
                var value = values[stride + i] / values[j + i];

                for (int l = i + 1; l < length2; l++)
                    values[stride + l] -= value * values[j + l];
            }
        }

        // Check for singular matrix
        if (Abs(values[2 * length * length - length - 1]) < tol)
        {
            result = null;
            return false;
        }

        var inverse = new double[length, length];

        for (int i = 0; i < length; i++)
        {
            for (int j = length - 1; j >= i; j--)
            {
                var stride = j * length2;
                var value = values[stride + length + i];

                for (int l = j + 1; l < length; l++)
                    value -= values[stride + l] * inverse[l, i];

                value /= values[stride + j];

                inverse[j, i] = value;
                inverse[i, j] = value;
            }
        }

        result = reverse ? transpose.Mult(inverse) : inverse.Mult(transpose);
        return true;
    }
}
