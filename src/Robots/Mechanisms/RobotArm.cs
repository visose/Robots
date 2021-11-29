using Rhino.Geometry;

namespace Robots
{
    public abstract partial class RobotArm : Mechanism
    {
        internal RobotArm(string model, Manufacturers manufactuer, double payload, Plane basePlane, Mesh? baseMesh, Joint[] joints) : base(model, manufactuer, payload, basePlane, baseMesh, joints, false) { }

        protected override void SetStartPlanes()
        {
            var kinematics = Kinematics(GetStartPose());
            for (int i = 0; i < Joints.Length; i++)
            {
                Plane plane = kinematics.Planes[i + 1];
                plane.Transform(Transform.PlaneToPlane(BasePlane, Plane.WorldXY));
                Joints[i].Plane = plane;
            }
        }

        protected abstract JointTarget GetStartPose();
    }
}