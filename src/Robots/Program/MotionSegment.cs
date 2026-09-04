using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

readonly record struct MotionSegment(SystemTarget Start, SystemTarget End, SystemTarget? Corner = null, SystemTarget? Next = null)
{
    public int TargetIndex => Corner?.Index ?? End.Index;

    public Target[] Lerp(RobotSystem robot, double time)
    {
        int count = Corner?.ProgramTargets.Count ?? End.ProgramTargets.Count;
        var targets = new Target[count];
        return Lerp(robot, time, targets);
    }

    internal Target[] Lerp(RobotSystem robot, double time, Target[] targets)
    {
        if (Corner is null)
            return End.Lerp(Start, robot, time, Start.TotalTime, End.TotalTime, targets);

        var next = Next ?? throw new InvalidOperationException("Fly-by segment is missing its next target.");
        ArgumentOutOfRangeException.ThrowIfNotEqual(targets.Length, Corner.ProgramTargets.Count, nameof(targets));
        double t = NormalizeTime(time);

        for (int group = 0; group < targets.Length; group++)
        {
            var corner = Corner.ProgramTargets[group];
            targets[group] = corner.Target.Zone.IsFlyBy
                ? CreateFlybyTarget(robot, corner, next.ProgramTargets[group], Start.ProgramTargets[group], End.ProgramTargets[group], t)
                : End.ProgramTargets[group].Lerp(Start.ProgramTargets[group], robot, time, Start.TotalTime, End.TotalTime);
        }

        return targets;
    }

    public int GetDivisions(RobotSystem robot, double linearStep, double angularStep)
    {
        int divisions = Corner is null ? 1 : 2;

        for (int group = 0; group < Start.ProgramTargets.Count; group++)
        {
            var corner = Corner?.ProgramTargets[group];
            var joints = robot.GetJoints(group);
            int current = corner is null || !corner.Target.Zone.IsFlyBy
                ? GetLineDivisions(Start.ProgramTargets[group], End.ProgramTargets[group], joints, linearStep, angularStep)
                : GetLineDivisions(Start.ProgramTargets[group], corner, joints, linearStep, angularStep)
                    + GetLineDivisions(corner, End.ProgramTargets[group], joints, linearStep, angularStep);
            divisions = Max(divisions, current);
        }

        return divisions;
    }

    double NormalizeTime(double time)
    {
        double duration = End.TotalTime - Start.TotalTime;

        return duration <= TimeTol
            ? 0.0
            : Clamp((time - Start.TotalTime) / duration, 0.0, 1.0);
    }

    static int GetLineDivisions(ProgramTarget start, ProgramTarget end, IReadOnlyList<Joint> joints, double linearStep, double angularStep)
    {
        double distance = start.WorldPlane.Origin.DistanceTo(end.WorldPlane.Origin);
        double divisions = distance / linearStep;

        for (int i = 0; i < joints.Count; i++)
        {
            double delta = Abs(end.Kinematics.Joints[i] - start.Kinematics.Joints[i]);
            double step = joints[i] is PrismaticJoint ? linearStep : angularStep;
            divisions = Max(divisions, delta / step);
        }

        return Max(1, (int)Ceiling(divisions));
    }

    static Target CreateFlybyTarget(RobotSystem robot, ProgramTarget corner, ProgramTarget next, ProgramTarget entry, ProgramTarget exit, double t)
    {
        int robotJointCount = robot.GetRobotJointCount(corner.Group);
        var entryJoints = entry.Kinematics.Joints;
        var cornerJoints = corner.Kinematics.Joints;
        var exitJoints = exit.Kinematics.Joints;
        var joints = new double[robotJointCount];
        GeometryUtil.Quadratic(entryJoints.AsSpan(0, robotJointCount), cornerJoints.AsSpan(0, robotJointCount), exitJoints.AsSpan(0, robotJointCount), joints, t);

        int externalCount = entryJoints.Length - robotJointCount;
        var external = externalCount == 0
            ? robot.GetInterpolationExternal(corner.Group, corner.Target, joints)
            : CreateExternal(entryJoints, cornerJoints, exitJoints, robotJointCount, externalCount, t);

        if (corner.IsJointMotion || next.IsJointMotion)
            return new JointTarget(joints, corner.Target, external);

        Plane entryPlane = entry.WorldPlane;
        Plane cornerPlane = corner.WorldPlane;
        Plane exitPlane = exit.WorldPlane;

        Plane plane = robot.CartesianLerp(entryPlane, exitPlane, t, 0.0, 1.0)
            .WithOrigin(GeometryUtil.Quadratic(entryPlane.Origin, cornerPlane.Origin, exitPlane.Origin, t));
        plane = corner.ToTargetPlane(plane);

        return new CartesianTarget(plane, corner.Target, motion: Motions.Linear, external: external);
    }

    static double[] CreateExternal(double[] entryJoints, double[] cornerJoints, double[] exitJoints, int robotJointCount, int externalCount, double t)
    {
        var external = new double[externalCount];
        GeometryUtil.Quadratic(entryJoints.AsSpan(robotJointCount), cornerJoints.AsSpan(robotJointCount), exitJoints.AsSpan(robotJointCount), external, t);
        return external;
    }
}
