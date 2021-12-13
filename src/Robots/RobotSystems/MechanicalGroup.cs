using System.Xml;
using System.Xml.Linq;
using Rhino.Geometry;

namespace Robots;

public class MechanicalGroup
{
    internal int Index { get; }
    internal string Name { get; }
    public RobotArm Robot { get; }
    public List<Mechanism> Externals { get; }
    public List<Joint> Joints { get; }
    public List<Plane> DefaultPlanes { get; }
    public List<Mesh>? DefaultMeshes { get; }

    internal MechanicalGroup(int index, List<Mechanism> mechanisms)
    {
        Index = index;
        Name = $"T_ROB{index + 1}";
        Joints = mechanisms.SelectMany(x => x.Joints.OrderBy(y => y.Number)).ToList();
        Robot = mechanisms.OfType<RobotArm>().FirstOrDefault();
        mechanisms.Remove(Robot);
        Externals = mechanisms;

        DefaultPlanes = mechanisms
            .Select(m => m.Joints.Select(j => j.Plane).Prepend(Plane.WorldXY)).SelectMany(p => p)
            .Append(Plane.WorldXY).Concat(Robot.Joints.Select(j => j.Plane).Append(Plane.WorldXY))
            .ToList();

        if (Robot.BaseMesh is not null)
        {
            string t = " Mesh shouldn't be null.";
            DefaultMeshes = mechanisms
                 .Select(m => m.Joints.Select(j => j.Mesh.NotNull(t)).Prepend(m.BaseMesh.NotNull(t))).SelectMany(p => p)
                 .Append(Robot.BaseMesh.NotNull(t)).Concat(Robot.Joints.Select(j => j.Mesh.NotNull(t)))
                 .ToList();
        }
    }

    internal static MechanicalGroup Create(XElement element, bool loadMeshes)
    {
        var index = element.GetIntAttributeOrDefault("group");
        var mechanisms = new List<Mechanism>();

        foreach (var mechanismElement in element.Elements())
            mechanisms.Add(Mechanism.Create(mechanismElement, loadMeshes));

        return new MechanicalGroup(index, mechanisms);
    }

    public KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? coupledPlane = null, Plane? basePlane = null) => new MechanicalGroupKinematics(this, target, prevJoints, coupledPlane, basePlane);

    public double DegreeToRadian(double degree, int i)
    {
        return i < 6
            ? Robot.DegreeToRadian(degree, i)
            : Externals.First(x => x.Joints.Contains(Joints.First(y => y.Number == i))).DegreeToRadian(degree, i);
    }

    public double RadianToDegree(double radian, int i)
    {
        return i < 6
            ? Robot.RadianToDegree(radian, i)
            : Externals.First(x => x.Joints.Contains(Joints.First(y => y.Number == i))).RadianToDegree(radian, i);
    }

    public double[] RadiansToDegreesExternal(Target target)
    {
        double[] values = new double[target.External.Length];

        foreach (var mechanism in Externals)
        {
            foreach (var joint in mechanism.Joints)
            {
                values[joint.Number - 6] = mechanism.RadianToDegree(target.External[joint.Number - 6], joint.Index);
            }
        }

        return values;
    }
}
