using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotUR : RobotArm
{
    internal RobotUR(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

    private protected override MechanismKinematics CreateSolver() => new OffsetWristKinematics(this);
    protected override JointTarget GetStartPose() => new(new double[] { 0, -HalfPI, 0, -HalfPI, 0, 0 });
    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);
}
