using Rhino.Geometry;

namespace Robots;

public abstract class RobotArm : Mechanism
{
    internal RobotArm(string model, Manufacturers manufacturer, double payload, MechanismBase mechanismBase, Joint[] joints)
        : base(model, manufacturer, payload, mechanismBase, joints, false) { }

    protected override void SetStartPlanes()
    {
        var thetas = Joints.Map(j => j.Theta);
        JointTarget startPose = new(thetas);
        var kinematics = Kinematics(startPose);

        for (int i = 0; i < Joints.Length; i++)
        {
            Plane plane = kinematics.Planes[i + 1];
            plane.InverseOrient(ref BasePlane);
            Joints[i].Plane = plane;
        }
    }

    protected abstract override double[]? DefaultAlpha { get; }
}
