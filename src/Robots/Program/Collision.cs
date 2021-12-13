using Rhino.Geometry;
using static System.Math;

namespace Robots;

#if NETSTANDARD2_0
public class Collision
{
    public bool HasCollision => throw NotImplemented();
    public Mesh[] Meshes => throw NotImplemented();
    public CellTarget CollisionTarget => throw NotImplemented();

    internal Collision(Program program, IEnumerable<int> first, IEnumerable<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep)
    {
        throw NotImplemented();
    }

    Exception NotImplemented() => new NotImplementedException(" Collisions have to be reimplemented.");
}

#elif NET48
public class Collision
{
    readonly Program _program;
    readonly RobotSystem _robotSystem;
    readonly double _linearStep;
    readonly double _angularStep;
    readonly IEnumerable<int> _first;
    readonly IEnumerable<int> _second;
    readonly Mesh? _environment;
    readonly int _environmentPlane;

    public Mesh[]? Meshes { get; private set; }
    public CellTarget? CollisionTarget { get; private set; }
    public bool HasCollision => CollisionTarget is not null;

    internal Collision(Program program, IEnumerable<int> first, IEnumerable<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep)
    {

        _program = program;
        _robotSystem = program.RobotSystem;
        _linearStep = linearStep;
        _angularStep = angularStep;
        _first = first;
        _second = second;
        _environment = environment;
        _environmentPlane = environmentPlane;

        Collide();
    }

    void Collide()
    {
        Parallel.ForEach(_program.Targets, (cellTarget, state) =>
        {
            if (cellTarget.Index == 0)
                return;

            var prevcellTarget = _program.Targets[cellTarget.Index - 1];

            double divisions = 1;

            int groupCount = cellTarget.ProgramTargets.Count;

            for (int group = 0; group < groupCount; group++)
            {
                var target = cellTarget.ProgramTargets[group];
                var prevTarget = prevcellTarget.ProgramTargets[group];

                double distance = prevTarget.WorldPlane.Origin.DistanceTo(target.WorldPlane.Origin);
                double linearDivisions = Ceiling(distance / _linearStep);

                double maxAngle = target.Kinematics.Joints.Zip(prevTarget.Kinematics.Joints, (x, y) => Abs(x - y)).Max();
                double angularDivisions = Ceiling(maxAngle / _angularStep);

                double tempDivisions = Max(linearDivisions, angularDivisions);
                if (tempDivisions > divisions) divisions = tempDivisions;
            }

            var meshes = new List<Mesh>();

            int j = (cellTarget.Index == 1) ? 0 : 1;

            for (int i = j; i < divisions; i++)
            {
                double t = (double)i / (double)divisions;
                var kineTargets = cellTarget.Lerp(prevcellTarget, _robotSystem, t, 0.0, 1.0);
                var kinematics = _program.RobotSystem.Kinematics(kineTargets);

                meshes.Clear();

                // TODO: Meshes not a property of KinematicSolution anymore
                // meshes.AddRange(kinematics.SelectMany(x => x.Meshes)); 
                var tools = cellTarget.ProgramTargets.Select(p => p.Target.Tool.Mesh).ToList();
                var robotMeshes = PoseMeshes(_program.RobotSystem, kinematics, tools);
                meshes.AddRange(robotMeshes);

                if (_environment is not null)
                {
                    if (_environmentPlane != -1)
                    {
                        Mesh currentEnvironment = _environment.DuplicateMesh();
                        currentEnvironment.Transform(Transform.PlaneToPlane(Plane.WorldXY, kinematics.SelectMany(x => x.Planes).ToList()[_environmentPlane]));
                        meshes.Add(currentEnvironment);
                    }
                    else
                    {
                        meshes.Add(_environment);
                    }
                }

                var setA = _first.Select(x => meshes[x]);
                var setB = _second.Select(x => meshes[x]);

                var meshClash = Rhino.Geometry.Intersect.MeshClash.Search(setA, setB, 1, 1);

                if (meshClash.Length > 0 && (CollisionTarget is null || CollisionTarget.Index > cellTarget.Index))
                {
                    Meshes = new Mesh[] { meshClash[0].MeshA, meshClash[0].MeshB };
                    CollisionTarget = cellTarget;
                    state.Break();
                }
            }
        });
    }

    static List<Mesh> PoseMeshes(RobotSystem robot, List<KinematicSolution> solutions, List<Mesh> tools)
    {
        if (robot is RobotCell cell)
        {
            var meshes = solutions.SelectMany((_, i) => PoseMeshes(cell.MechanicalGroups[i], solutions[i].Planes, tools[i])).ToList();
            return meshes;
        }
        else
        {
            var ur = (RobotCellUR)robot;
            var meshes = PoseMeshesRobot(ur.Robot, solutions[0].Planes, tools[0]);
            return meshes;
        }
    }

    static List<Mesh> PoseMeshes(MechanicalGroup group, IList<Plane> planes, Mesh tool)
    {
        planes = planes.ToList();
        var count = planes.Count - 1;
        planes.RemoveAt(count);
        planes.Add(planes[count - 1]);

        var outMeshes = group.DefaultMeshes.Select(m => m.DuplicateMesh()).Append(tool.DuplicateMesh()).ToList();

        for (int i = 0; i < group.DefaultPlanes.Count; i++)
        {
            var s = Transform.PlaneToPlane(group.DefaultPlanes[i], planes[i]);
            outMeshes[i].Transform(s);
        }

        return outMeshes;
    }

    static List<Mesh> PoseMeshesRobot(RobotArm arm, IList<Plane> planes, Mesh tool)
    {
        if (arm.BaseMesh is null)
            return new List<Mesh>(0);

        planes = planes.ToList();
        var count = planes.Count - 1;
        planes.RemoveAt(count);
        planes.Add(planes[count - 1]);

        var defaultPlanes = arm.Joints.Select(m => m.Plane).Prepend(arm.BasePlane).Append(Plane.WorldXY).ToList();
        var defaultMeshes = arm.Joints.Select(m => m.Mesh.NotNull("Joint mesh shouldn't be null.")).Prepend(arm.BaseMesh).Append(tool);
        var outMeshes = defaultMeshes.Select(m => m.DuplicateMesh()).ToList();

        for (int i = 0; i < defaultPlanes.Count; i++)
        {
            var s = Transform.PlaneToPlane(defaultPlanes[i], planes[i]);
            outMeshes[i].Transform(s);
        }

        return outMeshes;
    }
}

#endif