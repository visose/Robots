using Rhino.Geometry;

namespace Robots;

public class MechanicalGroup
{
    internal int Index { get; }
    internal string Name { get; }
    public RobotArm Robot { get; }
    public List<Mechanism> Externals { get; }
    public List<Joint> Joints { get; }

    readonly MechanicalGroupKinematics _kinematics;

    internal MechanicalGroup(int index, List<Mechanism> mechanisms)
    {
        Index = index;
        Name = $"T_ROB{index + 1}";
        Joints = mechanisms.SelectMany(x => x.Joints.OrderBy(y => y.Number)).ToList();
        Robot = mechanisms.OfType<RobotArm>().FirstOrDefault();
        mechanisms.Remove(Robot);
        Externals = mechanisms;

        _kinematics = new(this);
    }

    public KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? coupledPlane = null, Plane? basePlane = null) =>
        _kinematics.Solve(target, prevJoints, coupledPlane, basePlane);

    public double DegreeToRadian(double degree, int i)
    {
        return i < Robot.Joints.Length
            ? Robot.DegreeToRadian(degree, i)
            : Externals.First(x => x.Joints.Contains(Joints.First(y => y.Number == i))).DegreeToRadian(degree, i);
    }

    public double RadianToDegree(double radian, int i)
    {
        return i < Robot.Joints.Length
            ? Robot.RadianToDegree(radian, i)
            : Externals.First(x => x.Joints.Contains(Joints.First(y => y.Number == i))).RadianToDegree(radian, i);
    }

    public double[] RadiansToDegreesExternal(Target target)
    {
        int externalCount = Externals.Sum(e => e.Joints.Length);
        double[] values = new double[externalCount];
        int robotJointCount = Robot.Joints.Length;

        var external = target.External;
        Array.Resize(ref external, externalCount);

        foreach (var mechanism in Externals)
        {
            foreach (var joint in mechanism.Joints)
            {
                values[joint.Number - robotJointCount] = mechanism.RadianToDegree(external[joint.Number - robotJointCount], joint.Index);
            }
        }

        return values;
    }

    public double[] ExternalOrDefault(double[] external)
    {
        int externalCount = Externals.Sum(e => e.Joints.Length);
        Array.Resize(ref external, externalCount);
        return external;
    }
}
