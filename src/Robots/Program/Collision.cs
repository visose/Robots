using Rhino.Geometry;
#if RHINOCOMMON
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
    readonly int[] _first;
    readonly int[] _second;
    readonly Mesh? _environment;
    readonly int _environmentPlane;

    public Mesh[]? Meshes { get; private set; }
    public SystemTarget? CollisionTarget { get; private set; }
    public bool HasCollision => CollisionTarget is not null;

    internal Collision(Program program, IReadOnlyList<int> first, IReadOnlyList<int> second, Mesh? environment, int environmentPlane, double linearStep, double angularStep)
    {
        _program = program;
        _system = program.RobotSystem;
        _linearStep = CheckPositive(linearStep, nameof(linearStep));
        _angularStep = CheckPositive(angularStep, nameof(angularStep));
        _environment = environment;
        ArgumentOutOfRangeException.ThrowIfLessThan(environmentPlane, -1);
        _environmentPlane = environmentPlane;

        int meshCount = RhinoMeshPoser.CreateCollision(_program.RobotSystem).Meshes.Length + (_environment is null ? 0 : 1);
        _first = ValidateSet(first, nameof(first), meshCount);
        _second = ValidateSet(second, nameof(second), meshCount);
        ValidateEnvironmentPlane(environmentPlane);

        Collide();
    }

    static int[] ValidateSet(IReadOnlyList<int> indices, string name, int meshCount)
    {
        for (int i = 0; i < indices.Count; i++)
        {
            int index = indices[i];

            if (index < 0 || index >= meshCount)
                throw new ArgumentOutOfRangeException(name, index, $"Collision mesh index {index} is outside the available mesh range 0..{meshCount - 1}.");
        }

        return [.. indices];
    }

    void ValidateEnvironmentPlane(int environmentPlane)
    {
        if (_environment is null || _environmentPlane == -1)
            return;

        if (_program.Targets.Count == 0)
            throw new InvalidOperationException("Collision checking requires at least one target.");

        int planeCount = _program.Targets[0].Planes.Length;

        if (_environmentPlane >= planeCount)
            throw new ArgumentOutOfRangeException(nameof(environmentPlane), environmentPlane, $"Environment plane index is outside the posed robot plane range 0..{planeCount - 1}.");
    }

    static double CheckPositive(double value, string name)
    {
        _ = CheckFinite(value, name);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, name);
        return value;
    }

    void Collide()
    {
        var segments = _program.MotionSegments;
        object gate = new();
        int collisionPosition = int.MaxValue;

        _ = Parallel.For(0, segments.Count, (position, state) =>
        {
            lock (gate)
            {
                if (position >= collisionPosition)
                    return;
            }

            var segment = segments[position];
            int divisions = segment.GetDivisions(_linearStep, _angularStep);
            var meshPoser = RhinoMeshPoser.CreateCollision(_program.RobotSystem);
            int j = position == 0 ? 0 : 1;
            var prevJoints = segment.Start.JointSets();
            var targets = new Target[segment.Start.ProgramTargets.Count];

            for (int i = j; i <= divisions; i++)
            {
                double t = i / (double)divisions;
                double time = segment.Start.TotalTime + ((segment.End.TotalTime - segment.Start.TotalTime) * t);
                _ = segment.Lerp(_system, time, targets);
                var kinematics = _program.RobotSystem.Kinematics(targets, prevJoints);
                ThrowIfKinematicErrors(kinematics);
                prevJoints = kinematics.JointSets(prevJoints);

                var meshes = PoseMeshes(meshPoser, kinematics, _program.Targets[segment.TargetIndex]);

                var setA = SelectMeshes(meshes, _first);
                var setB = SelectMeshes(meshes, _second);

                var meshClash = Rhino.Geometry.Intersect.MeshClash.Search(setA, setB, 1, 1);

                if (meshClash.Length > 0)
                {
                    lock (gate)
                    {
                        if (position < collisionPosition)
                        {
                            Meshes = [meshClash[0].MeshA, meshClash[0].MeshB];
                            CollisionTarget = _program.Targets[segment.TargetIndex];
                            collisionPosition = position;
                        }
                    }

                    state.Break();
                }
            }
        });
    }

    Mesh[] PoseMeshes(RhinoMeshPoser meshPoser, IReadOnlyList<KinematicSolution> kinematics, SystemTarget systemTarget)
    {
        meshPoser.Pose(kinematics, systemTarget);
        var meshes = meshPoser.Meshes;

        if (_environment is null)
            return meshes;

        var result = new Mesh[meshes.Length + 1];
        Array.Copy(meshes, result, meshes.Length);
        result[^1] = _environmentPlane == -1
            ? _environment
            : _environment.DuplicateMesh();

        if (_environmentPlane != -1)
        {
            var plane = GetPlane(kinematics, _environmentPlane);
            _ = result[^1].Transform(plane.ToTransform());
        }

        return result;
    }

    static Mesh[] SelectMeshes(Mesh[] meshes, int[] indices)
    {
        var selected = new Mesh[indices.Length];

        for (int i = 0; i < selected.Length; i++)
            selected[i] = meshes[indices[i]];

        return selected;
    }

    static void ThrowIfKinematicErrors(IReadOnlyList<KinematicSolution> kinematics)
    {
        List<string>? errors = null;

        foreach (var solution in kinematics)
        {
            foreach (var error in solution.Errors)
                (errors ??= []).Add(error);
        }

        if (errors is not null)
            throw new InvalidOperationException($"Collision interpolation produced kinematic errors: {string.Join(" ", errors)}");
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
