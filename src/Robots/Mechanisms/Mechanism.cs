using Rhino.Geometry;

namespace Robots;

public abstract class Mechanism
{
    readonly string _model;
    Plane _basePlane;

    public Manufacturers Manufacturer { get; }
    public double Payload { get; }
    public ref Plane BasePlane => ref _basePlane;
    public Mesh BaseMesh { get; }
    public Joint[] Joints { get; }
    public bool MovesRobot { get; }
    public Mesh DisplayMesh { get; }
    public string Model => $"{Manufacturer}.{_model}";

    internal Mechanism(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh baseMesh, IEnumerable<Joint> joints, bool movesRobot)
    {
        _model = model;
        Manufacturer = manufacturer;
        Payload = payload;
        BasePlane = basePlane;
        BaseMesh = baseMesh;
        Joints = joints.TryCastArray();
        MovesRobot = movesRobot;

        DisplayMesh = CreateDisplayMesh();

        // Joints to radians
        for (int i = 0; i < Joints.Length; i++)
            Joints[i].Range = new Interval(DegreeToRadian(Joints[i].Range.T0, i), DegreeToRadian(Joints[i].Range.T1, i));

        SetStartPlanes();
    }

    Mesh CreateDisplayMesh()
    {
        var mesh = new Mesh();

        if (BaseMesh is null)
            return mesh;

        mesh.Append(BaseMesh);

        foreach (var joint in Joints)
            mesh.Append(joint.Mesh);

        mesh.Transform(BasePlane.ToTransform());
        return mesh;
    }

      public abstract KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null);

    protected abstract void SetStartPlanes();
    public abstract double DegreeToRadian(double degree, int i);
    public abstract double RadianToDegree(double radian, int i);
    public override string ToString() => $"{GetType().Name} ({Model})";
}
