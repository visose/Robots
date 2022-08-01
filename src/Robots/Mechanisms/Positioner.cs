using Rhino.Geometry;
using static System.Math;

namespace Robots;

public class Positioner : Mechanism
{
    internal Positioner(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, bool movesRobot)
        : base(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot) { }

    protected override void SetStartPlanes()
    {
        if (Joints.Length == 1)
        {
            Joints[0].Plane = new Plane(new Point3d(Joints[0].A, 0, Joints[0].D), Vector3d.XAxis, Vector3d.YAxis);
        }
        else
        {
            Joints[0].Plane = new Plane(new Point3d(0, 0, Joints[0].D), Vector3d.XAxis, Vector3d.ZAxis);
            Joints[1].Plane = new Plane(new Point3d(0, Joints[1].A, Joints[0].D + Joints[1].D), Vector3d.XAxis, Vector3d.YAxis);
        }
    }

    private protected override MechanismKinematics CreateSolver() => new PositionerKinematics(this);

    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180);
    public override double RadianToDegree(double radian, int i) => radian * (180 / PI);
}
