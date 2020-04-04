using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Xml.Linq;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    public abstract partial class RobotArm : Mechanism
    {
        internal RobotArm(string model, Manufacturers manufactuer, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, manufactuer, payload, basePlane, baseMesh, joints, false) { }

        protected override void SetStartPlanes()
        {
            var kinematics = Kinematics(GetStartPose());
            for (int i = 0; i < Joints.Length; i++)
            {
                Plane plane = kinematics.Planes[i + 1];
                plane.Transform(Transform.PlaneToPlane(this.BasePlane, Plane.WorldXY));
                Joints[i].Plane = plane;
            }
        }

        protected abstract JointTarget GetStartPose();
    }

    public class RobotAbb : RobotArm
    {
        internal RobotAbb(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturers.ABB, payload, basePlane, baseMesh, joints) { }

        protected override JointTarget GetStartPose() => new JointTarget(new double[] { 0, PI / 2, 0, 0, 0, 0 });

        public override KinematicSolution Kinematics(Target target, double[] prevJoints, Plane? basePlane = null) => new SphericalWristKinematics(this, target, prevJoints, basePlane);

        public static double ABBDegreeToRadian(double degree, int i)
        {
            double radian = degree.ToRadians();
            if (i == 1) radian = -radian + PI * 0.5;
            if (i == 2) radian *= -1;
            if (i == 4) radian *= -1;
            return radian;
        }

        public override double DegreeToRadian(double degree, int i)
        {
            return ABBDegreeToRadian(degree, i);
        }

        public override double RadianToDegree(double radian, int i)
        {
            if (i == 1) { radian -= PI * 0.5; radian = -radian; }
            if (i == 2) radian *= -1;
            if (i == 4) radian *= -1;
            return radian.ToDegrees();
        }
    }

    public class RobotKuka : RobotArm
    {
        internal RobotKuka(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturers.KUKA, payload, basePlane, baseMesh, joints) { }

        protected override JointTarget GetStartPose() => new JointTarget(new double[] { 0, PI / 2, 0, 0, 0, -PI });

        public override KinematicSolution Kinematics(Target target, double[] prevJoints, Plane? basePlane = null) => new SphericalWristKinematics(this, target, prevJoints, basePlane);

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree.ToRadians();
            if (i == 2) radian -= 0.5 * PI;
            radian = -radian;
            return radian;
        }

        public override double RadianToDegree(double radian, int i)
        {
            radian = -radian;
            if (i == 2) radian += 0.5 * PI;
            return radian.ToDegrees();
        }
    }

    public class RobotUR : RobotArm
    {
        internal RobotUR(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturers.UR, payload, basePlane, baseMesh, joints) { }

        public override KinematicSolution Kinematics(Target target, double[] prevJoints, Plane? basePlane = null) => new OffsetWristKinematics(this, target, prevJoints, basePlane);

        protected override JointTarget GetStartPose() => new JointTarget(new double[] { 0, -PI / 2, 0, -PI / 2, 0, 0 });
        public override double DegreeToRadian(double degree, int i) => degree * (PI / 180);
        public override double RadianToDegree(double radian, int i) => radian * (180 / PI);
    }

    public class RobotStaubli : RobotArm
    {
        internal RobotStaubli(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturers.Staubli, payload, basePlane, baseMesh, joints) { }

        protected override JointTarget GetStartPose() => new JointTarget(new double[] { 0, PI / 2, PI / 2, 0, 0, 0 });

        public override KinematicSolution Kinematics(Target target, double[] prevJoints, Plane? basePlane = null) => new SphericalWristKinematics(this, target, prevJoints, basePlane);

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree.ToRadians();
            if (i == 1) radian = -radian + PI * 0.5;
            if (i == 2) radian *= -1;
            if (i == 2) radian += PI * 0.5;
            if (i == 4) radian *= -1;
            return radian;
        }

        public override double RadianToDegree(double radian, int i)
        {
            if (i == 1) { radian -= PI * 0.5; radian = -radian; }
            if (i == 2) radian -= PI * 0.5;
            if (i == 2) radian *= -1;
            if (i == 4) radian *= -1;
            return radian.ToDegrees();
        }
    }
}