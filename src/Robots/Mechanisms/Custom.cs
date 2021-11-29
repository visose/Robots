using Rhino.Geometry;

namespace Robots
{
    public class Custom : Mechanism
    {
        internal Custom(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh? baseMesh, Joint[] joints, bool movesRobot)
            : base(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot) { }

        protected override void SetStartPlanes()
        {
            var plane = Plane.WorldXY;
            foreach (var joint in Joints)
            {
                joint.Plane = plane;
            }
        }

        public override double DegreeToRadian(double degree, int i) => degree;
        public override double RadianToDegree(double radian, int i) => radian;

        public override KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null)
            => new CustomKinematics(this, target, prevJoints, basePlane);

        class CustomKinematics : MechanismKinematics
        {
            internal CustomKinematics(Custom custom, Target target, double[]? prevJoints, Plane? basePlane)
                : base(custom, target, prevJoints, basePlane) { }

            protected override void SetJoints(Target target, double[]? prevJoints)
            {
                for (int i = 0; i < mechanism.Joints.Length; i++)
                {
                    int externalNum = mechanism.Joints[i].Number - 6;

                    if (target.External.Length < externalNum + 1)
                    {
                        Errors.Add($"Custom external axis not configured on this target.");
                    }
                    else
                    {
                        double value = target.External[externalNum];
                        Joints[i] = prevJoints == null ? value : value;
                    }
                }
            }

            protected override void SetPlanes(Target target)
            {
                for (int i = 0; i < Planes.Length; i++)
                {
                    Planes[i] = Plane.WorldXY;
                }
            }
        }
    }
}