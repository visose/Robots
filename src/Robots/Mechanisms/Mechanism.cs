using Rhino.Geometry;

namespace Robots;

public abstract class Mechanism
{
    Plane _basePlane;

    public Manufacturers Manufacturer { get; }
    public double Payload { get; }
    public ref Plane BasePlane => ref _basePlane;
    public Mesh BaseMesh { get; }
    public Mesh BaseCollisionMesh { get; }
    public Joint[] Joints { get; }
    public bool MovesRobot { get; }
    public Mesh DisplayMesh { get; }
    public string Model { get; }

    internal MechanismKinematics Solver { get => field ??= CreateSolver(); private set; }

    internal Mechanism(string model, Manufacturers manufacturer, double payload, MechanismBase mechanismBase, Joint[] joints, bool movesRobot)
    {
        Manufacturer = manufacturer;
        Model = $"{manufacturer}.{model}";
        Payload = payload;
        BasePlane = mechanismBase.Plane;
        BaseMesh = mechanismBase.Mesh;
        BaseCollisionMesh = mechanismBase.CollisionMesh;
        MovesRobot = movesRobot;
        Joints = joints;
        InitJoints();
        DisplayMesh = CreateDisplayMesh();

        SetStartPlanes();
    }

    void InitJoints()
    {
        var joints = Joints;

        var alphas = DefaultAlpha ?? new double[joints.Length];
        var thetas = DefaultTheta ?? new double[joints.Length];
        var signs = DefaultSign ?? [.. Enumerable.Repeat(1, joints.Length)];

        for (int i = 0; i < joints.Length; i++)
        {
            var joint = joints[i];

            if (joint is RevoluteJoint)
                joint.MaxSpeed = joint.MaxSpeed.ToRadians();

            joint.Alpha = double.IsNaN(joint.Alpha)
            ? alphas[i] : joint.Alpha.ToRadians();

            joint.Theta = double.IsNaN(joint.Theta)
                ? thetas[i] : joint.Theta.ToRadians();

            if (joint.Sign == 0)
                joint.Sign = signs[i];

            var range = joint.Range;
            range.T0 = DegreeToRadian(range.T0, i);
            range.T1 = DegreeToRadian(range.T1, i);
            range.MakeIncreasing();
            joint.Range = range;
        }
    }

    Mesh CreateDisplayMesh()
    {
        var mesh = new Mesh();

        if (BaseMesh is null)
            return mesh;

        mesh.Append(BaseMesh);

        foreach (var joint in Joints)
            mesh.Append(joint.Mesh);

        _ = mesh.Transform(BasePlane.ToTransform());
        return mesh;
    }

    public KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null) =>
        Solver.Solve(target, new(prevJoints), basePlane);

    internal KinematicSolution Kinematics(Target target, PreviousJoints prevJoints, Plane? basePlane) =>
        Solver.Solve(target, prevJoints, basePlane);

    private protected abstract MechanismKinematics CreateSolver();
    protected abstract void SetStartPlanes();
    protected virtual double[]? DefaultAlpha => null;
    protected virtual double[]? DefaultTheta => null;
    protected virtual int[]? DefaultSign => null;

    public abstract double DegreeToRadian(double degree, int i);
    public abstract double RadianToDegree(double radian, int i);

    public override string ToString() => $"{GetType().Name} ({Model})";
}
