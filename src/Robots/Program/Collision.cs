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

        var motionSamples = GetMotionSamples();
        int meshCount = new RhinoMeshPoser(_program.RobotSystem).Meshes.Length + (_environment is null ? 0 : 1);
        _first = ValidateSet(first, nameof(first), meshCount);
        _second = ValidateSet(second, nameof(second), meshCount);
        ValidateEnvironmentPlane(motionSamples, environmentPlane);

        Collide(motionSamples);
    }

    IReadOnlyList<SystemTarget> GetMotionSamples() =>
        _program.MotionSamples.Count > 0
            ? _program.MotionSamples
            : _program.Targets;

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

    void ValidateEnvironmentPlane(IReadOnlyList<SystemTarget> motionSamples, int environmentPlane)
    {
        if (_environment is null || _environmentPlane == -1)
            return;

        if (motionSamples.Count == 0)
            throw new InvalidOperationException("Collision checking requires at least one motion sample.");

        int planeCount = motionSamples[0].Planes.Length;

        if (_environmentPlane >= planeCount)
            throw new ArgumentOutOfRangeException(nameof(environmentPlane), environmentPlane, $"Environment plane index is outside the posed robot plane range 0..{planeCount - 1}.");
    }

    static double CheckPositive(double value, string name)
    {
        _ = CheckFinite(value, name);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, name);
        return value;
    }

    void Collide(IReadOnlyList<SystemTarget> motionSamples)
    {
        object gate = new();
        int collisionPosition = int.MaxValue;

        _ = Parallel.For(1, motionSamples.Count, (position, state) =>
        {
            lock (gate)
            {
                if (position >= collisionPosition)
                    return;
            }

            var systemTarget = motionSamples[position];
            var previous = motionSamples[position - 1];

            int divisions = GetDivisions(systemTarget, previous);

            RhinoMeshPoser meshPoser = new(_program.RobotSystem);

            int j = position == 1 ? 0 : 1;
            var prevJoints = previous.ProgramTargets.Map(x => x.Kinematics.Joints);

            for (int i = j; i <= divisions; i++)
            {
                double t = i / (double)divisions;
                var targets = systemTarget.Lerp(previous, _system, t, 0.0, 1.0);
                var kinematics = _program.RobotSystem.Kinematics(targets, prevJoints);
                ThrowIfKinematicErrors(kinematics);
                prevJoints = kinematics.Map(k => k.Joints);

                var meshes = PoseMeshes(meshPoser, kinematics, systemTarget);

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
                            CollisionTarget = systemTarget;
                            collisionPosition = position;
                        }
                    }

                    state.Break();
                }
            }
        });
    }

    int GetDivisions(SystemTarget systemTarget, SystemTarget previous)
    {
        int divisions = 1;

        for (int group = 0; group < systemTarget.ProgramTargets.Count; group++)
        {
            var target = systemTarget.ProgramTargets[group];
            var prevTarget = previous.ProgramTargets[group];

            double distance = prevTarget.WorldPlane.Origin.DistanceTo(target.WorldPlane.Origin);
            double linearDivisions = Ceiling(distance / _linearStep);
            double maxAngle = MaxJointDelta(target.Kinematics.Joints, prevTarget.Kinematics.Joints);
            double angularDivisions = Ceiling(maxAngle / _angularStep);
            int currentDivisions = (int)Max(linearDivisions, angularDivisions);
            divisions = Max(divisions, currentDivisions);
        }

        return divisions;
    }

    static double MaxJointDelta(double[] current, double[] previous)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(current.Length, previous.Length, nameof(current));

        double max = 0;

        for (int i = 0; i < current.Length; i++)
            max = Max(max, Abs(current[i] - previous[i]));

        return max;
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
        var errors = kinematics.SelectMany(solution => solution.Errors).ToArray();

        if (errors.Length > 0)
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
