using Rhino.Geometry;
#if RHINOCOMMON
using static System.Math;
using static Robots.Util;
#endif

namespace Robots;

#if RHINO3DM
public class Collision
{
    public bool HasCollision => throw NotSupported();
    public Mesh[] Meshes => throw NotSupported();
    public SystemTarget CollisionTarget => throw NotSupported();

    internal Collision(Program program, IReadOnlyList<int> first, IReadOnlyList<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep)
    {
        throw NotSupported();
    }

    static NotSupportedException NotSupported() => new("Collisions are not available when Robots is built against rhino3dm.");
}

#else
public class Collision
{
    readonly Program _program;
    readonly RobotSystem _system;
    readonly double _linearStep;
    readonly double _angularStep;
    readonly IReadOnlyList<int> _first;
    readonly IReadOnlyList<int> _second;
    readonly Mesh? _environment;
    readonly int _environmentPlane;

    public Mesh[]? Meshes { get; private set; }
    public SystemTarget? CollisionTarget { get; private set; }
    public bool HasCollision => CollisionTarget is not null;

    internal Collision(Program program, IReadOnlyList<int> first, IReadOnlyList<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep)
    {
        ValidateSet(first, nameof(first));
        ValidateSet(second, nameof(second));

        _program = program;
        _system = program.RobotSystem;
        _linearStep = CheckPositive(linearStep, nameof(linearStep));
        _angularStep = CheckPositive(angularStep, nameof(angularStep));
        _first = first;
        _second = second;
        _environment = environment;
        ArgumentOutOfRangeException.ThrowIfLessThan(environmentPlane, -1);
        _environmentPlane = environmentPlane;

        Collide();
    }

    static void ValidateSet(IReadOnlyList<int> indices, string name)
    {
        for (int i = 0; i < indices.Count; i++)
            ArgumentOutOfRangeException.ThrowIfNegative(indices[i], name);
    }

    static double CheckPositive(double value, string name)
    {
        _ = CheckFinite(value, name);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, name);
        return value;
    }

    void Collide()
    {
        _ = Parallel.ForEach(_program.Targets, (systemTarget, state) =>
        {
            if (systemTarget.Index == 0)
                return;

            var previous = _program.Targets[systemTarget.Index - 1];

            double divisions = 1;

            int groupCount = systemTarget.ProgramTargets.Count;

            for (int group = 0; group < groupCount; group++)
            {
                var target = systemTarget.ProgramTargets[group];
                var prevTarget = previous.ProgramTargets[group];

                double distance = prevTarget.WorldPlane.Origin.DistanceTo(target.WorldPlane.Origin);
                double linearDivisions = Ceiling(distance / _linearStep);

                double maxAngle = target.Kinematics.Joints.Zip(prevTarget.Kinematics.Joints, (x, y) => Abs(x - y)).Max();
                double angularDivisions = Ceiling(maxAngle / _angularStep);

                double tempDivisions = Max(linearDivisions, angularDivisions);
                if (tempDivisions > divisions) divisions = tempDivisions;
            }

            var meshPoser = new RhinoMeshPoser(_program.RobotSystem);

            int j = (systemTarget.Index == 1) ? 0 : 1;
            var prevJoints = previous.ProgramTargets.Map(x => x.Kinematics.Joints);

            for (int i = j; i < divisions; i++)
            {
                double t = i / (double)divisions;
                var targets = systemTarget.Lerp(previous, _system, t, 0.0, 1.0);
                var kinematics = _program.RobotSystem.Kinematics(targets, prevJoints);
                prevJoints = kinematics.Map(k => k.Joints);

                meshPoser.Pose(kinematics, systemTarget);
                var meshes = meshPoser.Meshes;

                if (_environment is not null)
                {
                    var temp = new Mesh[meshes.Length + 1];
                    Array.Copy(meshes, temp, meshes.Length);
                    temp[^1] = _environmentPlane == -1
                        ? _environment
                        : _environment.DuplicateMesh();

                    if (_environmentPlane != -1)
                    {
                        var plane = GetPlane(kinematics, _environmentPlane);
                        _ = temp[^1].Transform(plane.ToTransform());
                    }

                    meshes = temp;
                }

                var setA = _first.Select(x => meshes[x]);
                var setB = _second.Select(x => meshes[x]);

                var meshClash = Rhino.Geometry.Intersect.MeshClash.Search(setA, setB, 1, 1);

                if (meshClash.Length > 0 && (CollisionTarget is null || CollisionTarget.Index > systemTarget.Index))
                {
                    Meshes = [meshClash[0].MeshA, meshClash[0].MeshB];
                    CollisionTarget = systemTarget;
                    state.Break();
                }
            }
        });
    }

    static Plane GetPlane(IReadOnlyList<KinematicSolution> kinematics, int index)
    {
        foreach (var solution in kinematics)
        {
            if (index < solution.Planes.Length)
                return solution.Planes[index];

            index -= solution.Planes.Length;
        }

        throw new ArgumentOutOfRangeException(nameof(index), "Environment plane index is outside the posed robot planes.");
    }
}
#endif
