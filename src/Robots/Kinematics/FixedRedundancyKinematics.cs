using System.Runtime.CompilerServices;
using Rhino.Geometry;
using static System.Math;
using static Robots.GeometryMath;
using static Robots.Util;

namespace Robots;

/// <summary>
/// Analytical IK for a seven-revolute arm after fixing one redundant joint.
/// The remaining general 6R problem is solved with the Raghavan-Roth equations
/// and the Manocha-Canny matrix polynomial.
/// </summary>
class FixedRedundancyKinematics(RobotArm robot) : RobotKinematics(robot)
{
    const int Redundant = 2;
    const int JointCount = 7;
    const int ReducedJointCount = JointCount - 1;
    const double SupportAngleTolerance = 1e-10;
    const double SupportDistanceRelativeTolerance = 1e-12;
    const double RangeTolerance = 1e-10;
    const double DedupTolerance = 1e-6;
    const double PositionTolerance = 1e-5;
    const double OrientationTolerance = 1e-8;
    const double SingularRatioTolerance = 1e-5;

    static readonly string[] NearSingularityErrors = ["Target near singularity."];

    readonly Joint[] _joints = robot.Joints;
    readonly double _scale = GetScale(robot.Joints);
    readonly double _jacobianScale = GetChainLength(robot.Joints);

    public override bool CanSolve(RobotArm robot) => Supports(robot);
    public override int? RedundantJointIndex => Redundant;

    public static bool Supports(RobotArm robot)
    {
        var joints = robot.Joints;
        ReadOnlySpan<double> alpha = [0, -HalfPI, HalfPI, HalfPI, -HalfPI, HalfPI, HalfPI];

        if (!HasRevoluteDh(joints, alpha, SupportAngleTolerance))
            return false;

        foreach (var joint in joints)
        {
            if (joint.Sign != 1)
                return false;
        }

        double scale = GetScale(joints);
        double tolerance = Max(1, scale) * SupportDistanceRelativeTolerance;
        return double.IsFinite(scale)
            && scale > 0
            && Abs(joints[0].A) < tolerance
            && Abs(joints[1].A) < tolerance
            && Abs(joints[2].A) < tolerance
            && Abs(joints[5].A) < tolerance
            && Abs(joints[1].D) < tolerance
            && Abs(joints[3].D) < tolerance
            && Abs(joints[5].D) < tolerance
            && Abs(joints[3].A) > tolerance
            && Abs(joints[3].A + joints[4].A) < tolerance
            && Abs(joints[6].A) > tolerance
            && Abs(joints[0].D) > tolerance
            && Abs(joints[2].D) > tolerance
            && Abs(joints[4].D) > tolerance
            && Abs(joints[6].D) > tolerance;
    }

    protected override Transform[] ForwardKinematics(double[] joints) => ModifiedDH(joints);

    protected override bool TryGetConfiguration(
        double[] joints,
        out RobotConfigurations configuration)
    {
        configuration = RobotConfigurations.None;
        return true;
    }

    protected override InverseSolutions GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested)
    {
        double redundant = external.Length > 0
            ? external[0]
            : prevJoints.HasValue
                ? prevJoints[Redundant]
                : _joints[Redundant].Range.Mid;
        var candidates = GetSolutions(
            transform,
            redundant,
            prevJoints,
            preserveRedundant: external.Length > 0,
            out var errors);
        var solutions = new List<InverseSolution>(candidates.Count);

        foreach (var candidate in candidates)
            solutions.Add(new(candidate, RobotConfigurations.None));

        int selected = SelectSolution(
            solutions,
            requested,
            prevJoints,
            preserveWindings: true,
            out _);

        if (selected >= 0 && IsNearSingular(solutions[selected].Joints))
            solutions[selected] = solutions[selected] with { Errors = NearSingularityErrors };

        return new(solutions, errors, PreserveWindings: true);
    }

    public List<double[]> GetSolutions(
        Transform transform,
        double redundant,
        double[]? previous,
        out List<string> errors) =>
        GetSolutions(
            transform,
            redundant,
            new PreviousJoints(previous),
            preserveRedundant: true,
            out errors);

    List<double[]> GetSolutions(
        Transform transform,
        double redundant,
        PreviousJoints previous,
        bool preserveRedundant,
        out List<string> errors)
    {
        var chain = CreateChain(redundant);
        var target = RigidInverse(chain.Pre) * transform;
        target.M03 /= chain.Scale;
        target.M13 /= chain.Scale;
        target.M23 /= chain.Scale;

        var angles = SolveSixAxis(chain.Factors, target);
        var solutions = new List<double[]>(16);
        bool hasClosingSolution = false;

        foreach (var theta in angles)
        {
            var candidate = new double[JointCount];
            candidate[0] = theta[0];
            candidate[1] = theta[1];
            candidate[2] = redundant;
            candidate[3] = theta[2];
            candidate[4] = theta[3];
            candidate[5] = theta[4];
            candidate[6] = theta[5];

            if (!Closes(candidate, transform))
                continue;

            hasClosingSolution = true;

            if (!TryFitRanges(candidate, previous, preserveRedundant)
                || Contains(solutions, candidate))
            {
                continue;
            }

            solutions.Add(candidate);
        }

        errors = [];

        if (solutions.Count == 0)
        {
            errors.Add(hasClosingSolution
                ? "Target requires joints outside the permitted ranges."
                : "Target out of reach.");
        }

        return solutions;
    }

    static List<double[]> SolveSixAxis(
        ReadOnlySpan<Transform> factors,
        Transform target)
    {
        ReadOnlySpan<RaghavanRoth.Split> splits =
        [
            new(0, 1, 2, 4, 5, 3),
            new(1, 2, 3, 0, 5, 4),
            new(2, 3, 4, 0, 1, 5)
        ];
        var solutions = new List<double[]>(16);
        Span<Transform> samples = stackalloc Transform[RaghavanRoth.SampleTransformCount];
        Span<Transform> inverseSamples = stackalloc Transform[RaghavanRoth.SampleTransformCount];
        RaghavanRoth.BuildSampleTransforms(
            factors,
            samples,
            inverseSamples);

        foreach (var split in splits)
        {
            RaghavanRoth.Solve(
                factors,
                target,
                split,
                samples,
                inverseSamples,
                solutions);
        }

        return solutions;
    }

    ReducedChain CreateChain(double redundant)
    {
        // Modified DH factors exactly as Pre * Rz(q0)K0 * ... * Rz(q5)K5.
        // Fold the locked joint into the adjacent constant K factor.
        ReducedChain result = default;
        result.Pre = JointFrame(_joints[0]);
        var between = TranslationZ(_joints[0].D);
        int factor = 0;

        for (int i = 1; i < _joints.Length; i++)
        {
            var joint = _joints[i];

            if (i == Redundant)
            {
                between *= ModifiedDh(joint, redundant);
                continue;
            }

            between *= JointFrame(joint);
            result.Factors[factor++] = between;
            between = TranslationZ(joint.D);
        }

        result.Factors[factor] = between;
        double scale = _scale;

        for (int i = 0; i < ReducedJointCount; i++)
        {
            var transform = result.Factors[i];
            scale = Max(
                scale,
                Sqrt(
                    Square(transform.M03)
                    + Square(transform.M13)
                    + Square(transform.M23)));
        }

        for (int i = 0; i < ReducedJointCount; i++)
        {
            result.Factors[i].M03 /= scale;
            result.Factors[i].M13 /= scale;
            result.Factors[i].M23 /= scale;
        }

        result.Scale = scale;
        return result;
    }

    bool TryFitRanges(
        double[] joints,
        PreviousJoints previous,
        bool preserveRedundant)
    {
        for (int i = 0; i < joints.Length; i++)
        {
            var range = _joints[i].Range;

            if (preserveRedundant && i == Redundant)
            {
                if (joints[i] < range.T0 - RangeTolerance
                    || joints[i] > range.T1 + RangeTolerance)
                {
                    return false;
                }

                continue;
            }

            double angle = NormalizeAngle(joints[i]);
            int minTurn = (int)Ceiling((range.T0 - angle - RangeTolerance) / PI2);
            int maxTurn = (int)Floor((range.T1 - angle + RangeTolerance) / PI2);

            if (minTurn > maxTurn)
                return false;

            int turn = previous.HasValue
                ? Clamp((int)Round((previous[i] - angle) / PI2), minTurn, maxTurn)
                : minTurn;
            joints[i] = angle + turn * PI2;
        }

        return true;
    }

    bool Closes(double[] joints, Transform target)
    {
        var actual = Transform.Identity;

        for (int i = 0; i < joints.Length; i++)
            actual *= ModifiedDh(_joints[i], joints[i]);

        double positionErrorSquared =
            Square(actual.M03 - target.M03)
            + Square(actual.M13 - target.M13)
            + Square(actual.M23 - target.M23);

        if (positionErrorSquared > Square(PositionTolerance))
            return false;

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                if (Abs(actual[row, column] - target[row, column]) > OrientationTolerance)
                    return false;
            }
        }

        return true;
    }

    bool IsNearSingular(double[] joints)
    {
        Span<Transform> frames = stackalloc Transform[ReducedJointCount];
        var transform = Transform.Identity;
        int variable = 0;

        for (int i = 0; i < joints.Length; i++)
        {
            var joint = _joints[i];

            if (i != Redundant)
                frames[variable++] = transform * JointFrame(joint);

            transform *= ModifiedDh(joint, joints[i]);
        }

        Span<double> jacobian = stackalloc double[ReducedJointCount * ReducedJointCount];
        double endX = transform.M03;
        double endY = transform.M13;
        double endZ = transform.M23;
        double inverseScale = 1 / _jacobianScale;

        for (int column = 0; column < ReducedJointCount; column++)
        {
            var frame = frames[column];
            double originX = frame.M03;
            double originY = frame.M13;
            double originZ = frame.M23;
            double axisX = frame.M02;
            double axisY = frame.M12;
            double axisZ = frame.M22;
            double offsetX = endX - originX;
            double offsetY = endY - originY;
            double offsetZ = endZ - originZ;
            jacobian[column] = (axisY * offsetZ - axisZ * offsetY) * inverseScale;
            jacobian[ReducedJointCount + column] = (axisZ * offsetX - axisX * offsetZ) * inverseScale;
            jacobian[ReducedJointCount * 2 + column] = (axisX * offsetY - axisY * offsetX) * inverseScale;
            jacobian[ReducedJointCount * 3 + column] = axisX;
            jacobian[ReducedJointCount * 4 + column] = axisY;
            jacobian[ReducedJointCount * 5 + column] = axisZ;
        }

        return JacobianCondition.MinimumSingularRatio(jacobian)
            <= SingularRatioTolerance;
    }

    static bool Contains(List<double[]> solutions, double[] candidate)
    {
        foreach (var solution in solutions)
        {
            bool equal = true;

            for (int i = 0; i < candidate.Length; i++)
            {
                if (Abs(NormalizeAngle(solution[i] - candidate[i])) > DedupTolerance)
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

    static Transform JointFrame(Joint joint)
    {
        var (sine, cosine) = SinCos(joint.Alpha);
        Transform result = default;
        result.Set(
            1, 0, 0, joint.A,
            0, cosine, -sine, 0,
            0, sine, cosine, 0);
        return result;
    }

    static Transform TranslationZ(double distance)
    {
        var result = Transform.Identity;
        result.M23 = distance;
        return result;
    }

    static Transform ModifiedDh(Joint joint, double theta)
    {
        var (sineTheta, cosineTheta) = SinCos(theta);
        var (sineAlpha, cosineAlpha) = SinCos(joint.Alpha);
        Transform result = default;
        result.Set(
            cosineTheta, -sineTheta, 0, joint.A,
            sineTheta * cosineAlpha, cosineTheta * cosineAlpha, -sineAlpha, -joint.D * sineAlpha,
            sineTheta * sineAlpha, cosineTheta * sineAlpha, cosineAlpha, joint.D * cosineAlpha);
        return result;
    }

    static double GetScale(Joint[] joints)
    {
        double scale = 0;

        foreach (var joint in joints)
            scale = Max(scale, Hypot(joint.A, joint.D));

        return scale;
    }

    static double Square(double value) => value * value;

    [InlineArray(ReducedJointCount)]
    struct TransformBuffer
    {
        Transform _element;
    }

    struct ReducedChain
    {
        public TransformBuffer Factors;
        public Transform Pre;
        public double Scale;
    }
}
