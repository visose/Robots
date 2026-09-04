using System.Diagnostics.CodeAnalysis;
using Rhino.Geometry;
using static System.Math;
using static Robots.GeometryMath;

namespace Robots;

readonly record struct NumericalKinematicsSettings(bool UseModifiedDH, int? Redundant);

class NumericalKinematics(RobotArm robot, bool useModifiedDH, int? redundant) : RobotKinematics(robot)
{
    readonly int _jointCount = robot.Joints.Length;
    readonly bool _useModifiedDH = useModifiedDH;
    readonly int? _redundant = redundant;
    readonly double[] _midJoints = robot.Joints.Map(j => j.Range.Mid);

    public NumericalKinematics(RobotArm robot)
        : this(robot, robot.NumericalSettings.UseModifiedDH, robot.NumericalSettings.Redundant) { }

    public override bool CanSolve(RobotArm robot)
    {
        for (int i = 0; i < robot.Joints.Length; i++)
        {
            if (robot.Joints[i] is not RevoluteJoint)
                return false;
        }

        return true;
    }

    public override bool RequiresContinuation => true;
    public override int? RedundantJointIndex => _redundant;

    protected override InverseSolutions GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested)
    {
        var joints = SolveInverse(transform, external, prevJoints, out var errors);
        return new(
            [new(joints, RobotConfigurations.None, errors)],
            []);
    }

    protected override bool TryGetConfiguration(
        double[] joints,
        out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.None;
        return true;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        if (_useModifiedDH)
            return ModifiedDH(joints);

        return DH(joints);
    }

    double[] SolveInverse(Transform t, double[] external, PreviousJoints prevJoints, out List<string> errors)
    {
        const double max = 0.3;

        errors = [];
        var previous = prevJoints.HasValue ? prevJoints.Values : _midJoints.AsSpan();
        var joints = previous.ToArray();
        var error = new double[6];

        if (_redundant is int redundant)
            joints[redundant] = external.Length > 0 ? external[0] : previous[redundant];

        for (int i = 0; i < 400; i++)
        {
            Transform forward = default;
            double[,]? inverse = null;

            for (int ii = 0; ii < 20; ii++)
            {
                forward = Forward(joints);
                Subtract(ref forward, ref t, error);

                if (Converged(error))
                    return joints;

                var jacobian = Jacobian(joints, ref forward);

                if (TryPseudoInverse(jacobian, out var candidate))
                {
                    inverse = candidate;
                    break;
                }

                for (int j = 0; j < joints.Length; j++)
                {
                    if (j != _redundant)
                        joints[j] += 1e-3;
                }
            }

            if (inverse is null)
            {
                errors.Add("Target near singularity.");
                return joints;
            }

            var transpose = inverse.Transpose();
            var deltas = error.Mult(transpose);
            var maxValue = deltas.Max(Abs);

            if (maxValue > max)
            {
                var value = maxValue / max;

                for (int j = 0; j < deltas.Length; j++)
                    deltas[j] /= value;
            }

            for (int j = 0; j < deltas.Length; j++)
                joints[j] += j == _redundant ? 0 : deltas[j];
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

    static bool Converged(double[] error)
    {
        Vector3d position = new(error[0], error[1], error[2]);
        Vector3d rotation = new(error[3], error[4], error[5]);
        return position.SquareLength <= 1e-10 && rotation.SquareLength <= 1e-16;
    }

    static void Subtract(ref Transform f, ref Transform t, double[] error)
    {
        var relative = t * RigidInverse(f);
        var rotation = relative.RotationVector();
        error[0] = t.M03 - f.M03;
        error[1] = t.M13 - f.M13;
        error[2] = t.M23 - f.M23;
        error[3] = rotation.X;
        error[4] = rotation.Y;
        error[5] = rotation.Z;
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
