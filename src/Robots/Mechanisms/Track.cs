using Rhino.Geometry;

namespace Robots;

public class Track : Mechanism
{
    internal Track(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, bool movesRobot) : base(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot) { }

    protected override void SetStartPlanes()
    {
        var plane = Plane.WorldXY;
        foreach (var joint in Joints)
        {
            plane.Origin = plane.Origin + plane.XAxis * joint.A + plane.ZAxis * joint.D;
            joint.Plane = plane;
        }

        /*
        if (Joints.Length == 3)
        {
            plane = Joints[2].Plane;
            plane.Rotate(PI, plane.XAxis);
            Joints[2].Plane = plane;
        }
        */
    }

    public override double DegreeToRadian(double degree, int i) => degree;
    public override double RadianToDegree(double radian, int i) => radian;

    public override KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null) => new TrackKinematics(this, target, basePlane);
}
