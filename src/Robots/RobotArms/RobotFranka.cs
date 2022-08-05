using Rhino.Geometry;
using static System.Math;

namespace Robots;

public class RobotFranka : RobotArm
{
    internal RobotFranka(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

    private protected override MechanismKinematics CreateSolver() => new FrankaNumericalKinematics(this);
    protected override JointTarget GetStartPose() => new(new[] { 0.0, 0, 0, 0, 0, 0, 0 });

    public override double DegreeToRadian(double degree, int i) => degree * (PI / 180.0);
    public override double RadianToDegree(double radian, int i) => radian * (180.0 / PI);
}
