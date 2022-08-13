using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots;

public class RobotDoosan : RobotArm
{
    internal readonly double[] Start = new[] { 0, HalfPI, HalfPI, 0, 0, PI };

    internal RobotDoosan(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints)
        : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

    private protected override MechanismKinematics CreateSolver() => new SphericalWristKinematics(this);
    protected override JointTarget GetStartPose() => new(Start);

    public override double DegreeToRadian(double degree, int i)
    {
        var radian = degree.ToRadians();
        if (i == 0) radian -= PI;
        return (radian + Start[i]);
    }

    public override double RadianToDegree(double radian, int i)
    {
        if (i == 0) radian += PI;
        return (radian - Start[i]) * (180.0 / PI);
    }
}
