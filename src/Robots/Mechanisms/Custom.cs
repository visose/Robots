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
                joint.Plane = plane;
        }

        public override double DegreeToRadian(double degree, int i) => degree;
        public override double RadianToDegree(double radian, int i) => radian;

        public override KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null)
            => new CustomKinematics(this, target, prevJoints, basePlane);
    }
}