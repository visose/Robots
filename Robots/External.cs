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
    public class Positioner : Mechanism
    {
        internal Positioner(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, bool movesRobot) : base(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot) { }

        protected override void SetStartPlanes()
        {
            if (Joints.Length == 1)
            {
                Joints[0].Plane = new Plane(new Point3d(Joints[0].A, 0, Joints[0].D), Vector3d.XAxis, Vector3d.YAxis);
            }
            else
            {
                Joints[0].Plane = new Plane(new Point3d(0, 0, Joints[0].D), Vector3d.XAxis, Vector3d.ZAxis);
                Joints[1].Plane = new Plane(new Point3d(0, Joints[1].A, Joints[0].D + Joints[1].D), Vector3d.XAxis, Vector3d.YAxis);
            }
        }

        public override double DegreeToRadian(double degree, int i) => degree * (PI / 180);
        public override double RadianToDegree(double radian, int i) => radian * (180 / PI);

        public override KinematicSolution Kinematics(Target target, double[] prevJoints = null, Plane? basePlane = null) => new PositionerKinematics(this, target, prevJoints, basePlane);

        class PositionerKinematics : MechanismKinematics
        {
            internal PositionerKinematics(Positioner positioner, Target target, double[] prevJoints, Plane? basePlane) : base(positioner, target, prevJoints, basePlane) { }

            protected override void SetJoints(Target target, double[] prevJoints)
            {
                for (int i = 0; i < mechanism.Joints.Length; i++)
                {
                    int externalNum = mechanism.Joints[i].Number - 6;
                    if (target.External.Length - 1 < externalNum)
                        Errors.Add($"Positioner external axis not configured on this target.");
                    else
                        Joints[i] = target.External[externalNum];
                }
                if (prevJoints != null)
                    Joints = JointTarget.GetAbsoluteJoints(Joints, prevJoints);
            }

            protected override void SetPlanes(Target target)
            {
                int jointCount = mechanism.Joints.Length;

                for (int i = 0; i < jointCount; i++)
                {
                    Planes[i + 1] = mechanism.Joints[i].Plane;
                    for (int j = i; j >= 0; j--)
                        Planes[i + 1].Rotate(Joints[j], mechanism.Joints[j].Plane.Normal, mechanism.Joints[j].Plane.Origin);
                }
            }
        }
    }

    public class Custom : Mechanism
    {
        internal Custom(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, bool movesRobot)
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

        public override KinematicSolution Kinematics(Target target, double[] prevJoints = null, Plane? basePlane = null)
            => new CustomKinematics(this, target, prevJoints, basePlane);

        class CustomKinematics : MechanismKinematics
        {
            internal CustomKinematics(Custom custom, Target target, double[] prevJoints, Plane? basePlane)
                : base(custom, target, prevJoints, basePlane) { }

            protected override void SetJoints(Target target, double[] prevJoints)
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

        public override KinematicSolution Kinematics(Target target, double[] prevJoints = null, Plane? basePlane = null) => new TrackKinematics(this, target, basePlane);

        class TrackKinematics : MechanismKinematics
        {
            internal TrackKinematics(Track track, Target target, Plane? basePlane) : base(track, target, null, basePlane) { }

            protected override void SetJoints(Target target, double[] prevJoints)
            {
                for (int i = 0; i < mechanism.Joints.Length; i++)
                {
                    int externalNum = mechanism.Joints[i].Number - 6;
                    if (target.External.Length < externalNum + 1) Errors.Add($"Track external axis not configured on this target.");
                    else Joints[i] = target.External[externalNum];
                }
            }

            protected override void SetPlanes(Target target)
            {
                Planes[1] = mechanism.Joints[0].Plane;
                Planes[1].Origin += Planes[1].XAxis * Joints[0];
                if (mechanism.Joints.Length == 1) return;

                Planes[2] = mechanism.Joints[1].Plane;
                Planes[2].Origin += Planes[1].Origin + Planes[2].YAxis * Joints[1];
                if (mechanism.Joints.Length == 2) return;

                Planes[3] = mechanism.Joints[2].Plane;
                Planes[3].Origin += Planes[2].Origin + Planes[3].ZAxis * Joints[2];
                if (mechanism.Joints.Length == 3) return;
            }
        }
    }
}