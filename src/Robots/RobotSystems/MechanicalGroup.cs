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
        double[] values = new double[target.External.Length];
        int jointCount = Robot.Joints.Length;

        foreach (var mechanism in Externals)
        {
            foreach (var joint in mechanism.Joints)
            {
                values[joint.Number - jointCount] = mechanism.RadianToDegree(target.External[joint.Number - jointCount], joint.Index);
            }
        }

        return values;
    }
}
