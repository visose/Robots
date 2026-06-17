using Rhino.Geometry;

namespace Robots;

public class MechanicalGroup
{
    internal int Index { get; }
    internal string Name { get; }
    public RobotArm Robot { get; }
    public Mechanism[] Externals { get; }
    public Joint[] Joints { get; }
    public int RobotJointCount => Robot.Joints.Length;
    public int ExternalJointCount => Joints.Length - RobotJointCount;
    internal int PlaneCount => Joints.Length + Externals.Length + 2;
    internal int RobotFlangePlaneIndex => PlaneCount - 2;

    readonly MechanicalGroupKinematics _kinematics;

    internal MechanicalGroup(int index, List<Mechanism> mechanisms)
    {
        Index = index;
        Name = $"T_ROB{index + 1}";
        Joints = [.. mechanisms.SelectMany(x => x.Joints.OrderBy(y => y.Number))];
        RobotArm? robot = null;

        foreach (var mechanism in mechanisms.OfType<RobotArm>())
        {
            if (robot is not null)
                throw new ArgumentException("Mechanical groups must contain exactly one robot arm.", nameof(mechanisms));

            robot = mechanism;
        }

        Robot = robot ?? throw new ArgumentException("Mechanical groups must contain exactly one robot arm.", nameof(mechanisms));
        Externals = [.. mechanisms.Where(m => !ReferenceEquals(m, Robot))];

        _kinematics = new(this);
    }

    internal KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? coupledPlane = null, Plane? basePlane = null) =>
        _kinematics.Solve(target, prevJoints, coupledPlane, basePlane);

    internal int ExternalFlangePlaneIndex(int mechanismIndex)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(mechanismIndex);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(mechanismIndex, Externals.Length);

        int index = -1;

        for (int i = 0; i <= mechanismIndex; i++)
            index += Externals[i].Joints.Length + 1;

        return index;
    }

    internal double DegreeToRadian(double degree, int i)
    {
        return i < RobotJointCount
            ? Robot.DegreeToRadian(degree, i)
            : GetExternal(i).DegreeToRadian(degree, i);
    }

    internal double RadianToDegree(double radian, int i)
    {
        return i < RobotJointCount
            ? Robot.RadianToDegree(radian, i)
            : GetExternal(i).RadianToDegree(radian, i);
    }

    Mechanism GetExternal(int jointNumber)
    {
        foreach (var external in Externals)
        {
            foreach (var joint in external.Joints)
            {
                if (joint.Number == jointNumber)
                    return external;
            }
        }

        throw new ArgumentOutOfRangeException(nameof(jointNumber), "Joint number is outside the mechanical group.");
    }

    internal double[] RadiansToDegreesExternal(Target target)
    {
        int externalCount = ExternalJointCount;
        double[] values = new double[externalCount];
        int robotJointCount = RobotJointCount;

        foreach (var mechanism in Externals)
        {
            foreach (var joint in mechanism.Joints)
            {
                int index = joint.Number - robotJointCount;
                double value = index < target.External.Length ? target.External[index] : 0;
                values[index] = mechanism.RadianToDegree(value, joint.Index);
            }
        }

        return values;
    }
}
