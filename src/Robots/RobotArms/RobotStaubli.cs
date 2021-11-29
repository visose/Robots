using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots
{
    public class RobotStaubli : RobotArm
    {
        internal RobotStaubli(string model, double payload, Plane basePlane, Mesh? baseMesh, Joint[] joints) : base(model, Manufacturers.Staubli, payload, basePlane, baseMesh, joints) { }

        protected override JointTarget GetStartPose() => new JointTarget(new double[] { 0, PI / 2, PI / 2, 0, 0, 0 });

        public override KinematicSolution Kinematics(Target target, double[]? prevJoints, Plane? basePlane = null) => new SphericalWristKinematics(this, target, prevJoints, basePlane);

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree.ToRadians();
            if (i == 1) radian = -radian + HalfPI;
            if (i == 2) radian *= -1;
            if (i == 2) radian += HalfPI;
            if (i == 4) radian *= -1;
            return radian;
        }

        public override double RadianToDegree(double radian, int i)
        {
            if (i == 1) { radian -= HalfPI; radian = -radian; }
            if (i == 2) radian -= HalfPI;
            if (i == 2) radian *= -1;
            if (i == 4) radian *= -1;
            return radian.ToDegrees();
        }
    }
}