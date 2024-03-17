using Rhino.Geometry;
using K = Kdl;
using static Kdl.Joint;

namespace Robots;

class FrankaKdlKinematics : RobotKinematics, IDisposable
{
    const int _redundant = 2;

    readonly List<string> _empty = [];
    readonly double[] _midJoints;
    readonly K.JntArray _qMin;
    readonly K.JntArray _qMax;
    readonly K.Chain _chain;
    readonly K.ChainIkSolverVel_pinv _velSolver;
    readonly K.ChainFkSolverPos_recursive _fkSolver;
    readonly K.ChainIkSolverPos_NR_JL _ikSolver;

    public FrankaKdlKinematics(RobotArm robot)
        : base(robot)
    {
        var joints = _mechanism.Joints;
        _midJoints = joints.Map(j => j.Range.Mid);
        _qMin = ToJntArray(joints.Map(j => double.MinValue));
        _qMax = ToJntArray(joints.Map(j => double.MaxValue));

        _chain = CreateChain();

        _velSolver = new(_chain, 1e-5, 150);
        _fkSolver = new(_chain);
        _ikSolver = new(_chain, _qMin, _qMax, _fkSolver, _velSolver, 100, 1e-4);
    }

    K.Chain CreateChain()
    {
        var j = _mechanism.Joints;
        K.Chain chain = new();

        AddSegment(JointType._None, j[0].A, j[0].Alpha, j[0].D, 0);
        AddSegment(JointType.RotZ, j[1].A, j[1].Alpha, j[1].D, 0);
        AddSegment(JointType.RotZ, j[2].A, j[2].Alpha, j[2].D, 0);
        AddSegment(JointType.RotZ, j[3].A, j[3].Alpha, j[3].D, 0);
        AddSegment(JointType.RotZ, j[4].A, j[4].Alpha, j[4].D, 0);
        AddSegment(JointType.RotZ, j[5].A, j[5].Alpha, j[5].D, 0);
        AddSegment(JointType.RotZ, j[6].A, j[6].Alpha, 0, 0);
        AddSegment(JointType.RotZ, 0, 0, j[6].D, 0);
        return chain;

        void AddSegment(JointType jointType, double a, double z, double d, double t)
        {
            using K.Joint joint = new(jointType);
            using var frame = K.Frame.DH_Craig1989(a, z, d, t);
            using K.Segment segment = new(joint, frame);
            chain.addSegment(segment);
        }
    }

    protected override int SolutionCount => 1;

    protected override double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        errors = _empty;

        var current = prevJoints ?? _midJoints;

        double? redundantValue =
            external.Length > 0 ? external[0] : prevJoints?[_redundant];

        if (redundantValue is not null)
        {
            _qMin.set(_redundant, redundantValue ?? double.MinValue);
            _qMax.set(_redundant, redundantValue ?? double.MaxValue);
        }

        _ikSolver.setJointLimits(_qMin, _qMax);

        using var q_init = ToJntArray(current);
        using K.Frame pos_goal = ToFrame(transform);
        using K.JntArray q_sol = new(7);
        var retVal = _ikSolver.CartToJnt(q_init, pos_goal, q_sol);

        if (retVal != 0)
        {
            var text = _ikSolver.strError(retVal);

            errors =
            [
                $"Warning: Target unreachable ({text})"
            ];
        }

        var vals = FromJntArray(q_sol);
        return vals;
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        using var ja = ToJntArray(joints);

        var _ts = new Transform[joints.Length];
        using K.FrameVector frames = new(joints.Length + 1);

        for (int i = 0; i < frames.Capacity; i++)
            frames.Add(new());

        var kinematics_status = _fkSolver.JntToCart(ja, frames);

        for (int i = 0; i < joints.Length; i++)
        {
            using var frame = frames[i + 1];
            using var p = frame.p;
            using var r = frame.M;
            using var vx = r.UnitX();
            using var vy = r.UnitY();

            Plane plane = new((Point3d)ToVector3d(p), ToVector3d(vx), ToVector3d(vy));
            _ts[i] = plane.ToTransform();
        }

        return _ts;
    }

    static Vector3d ToVector3d(K.Vector v) => new(v.x(), v.y(), v.z());

    static K.Frame ToFrame(Transform t)
    {
        using K.Rotation rotation = new(
            t.M00, t.M01, t.M02,
            t.M10, t.M11, t.M12,
            t.M20, t.M21, t.M22
            );

        using K.Vector vector = new(t.M03, t.M13, t.M23);
        return new(rotation, vector);
    }

    static double[] FromJntArray(K.JntArray ja)
    {
        int count = (int)ja.rows();
        var vals = new double[count];

        for (int i = 0; i < count; i++)
            vals[i] = ja.get(i);

        return vals;
    }

    static K.JntArray ToJntArray(double[] joints)
    {
        var ja = new K.JntArray((uint)joints.Length);

        for (int i = 0; i < joints.Length; i++)
            ja.set(i, joints[i]);

        return ja;
    }

    bool _isDisposed;

    ~FrankaKdlKinematics() => Dispose(false);

    public void Dispose()
    {
        Dispose(true);
        GC.SuppressFinalize(this);
    }

    void Dispose(bool disposing)
    {
        if (_isDisposed)
            return;

        _isDisposed = disposing;

        _qMin.Dispose();
        _qMax.Dispose();
        _chain.Dispose();
        _velSolver.Dispose();
        _fkSolver.Dispose();
        _ikSolver.Dispose();
    }
}
