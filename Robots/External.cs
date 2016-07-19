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
            Joints[0].Plane = new Plane(new Point3d(0, 0, Joints[0].D), Vector3d.YAxis, Vector3d.ZAxis);
            if (Joints.Length > 1)
                Joints[1].Plane = new Plane(new Point3d(Joints[1].A, 0, Joints[0].D + Joints[1].D), Vector3d.YAxis, -Vector3d.XAxis);
        }

        public override double DegreeToRadian(double degree, int i) => degree * (PI / 180);
        public override double RadianToDegree(double radian, int i) => radian * (180 / PI);

        public override KinematicSolution Kinematics(Target target, double[] prevJoints = null, bool displayMeshes = false, Plane? basePlane = null) => new PositionerKinematics(this, target, prevJoints, displayMeshes, basePlane);

        class PositionerKinematics : MechanismKinematics
        {
            internal PositionerKinematics(Positioner positioner, Target target, double[] prevJoints, bool displayMeshes, Plane? basePlane) : base(positioner, target, prevJoints, displayMeshes, basePlane) { }

            protected override void SetJoints(Target target, double[] prevJoints)
            {
                for (int i = 0; i < mechanism.Joints.Length; i++)
                {
                    int externalNum = mechanism.Joints[i].Number - 6;
                    if (target.External == null || target.External.Length - 1 < externalNum)
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

    public class Track : Mechanism
    {
        internal Track(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, bool movesRobot) : base(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot) { }

        protected override void SetStartPlanes()
        {
            Joints[0].Plane = new Plane(new Point3d(0, Joints[0].A, Joints[0].D), Vector3d.XAxis, Vector3d.YAxis);
        }

        public override double DegreeToRadian(double degree, int i) => degree;
        public override double RadianToDegree(double radian, int i) => radian;

        public override KinematicSolution Kinematics(Target target, double[] prevJoints = null, bool displayMeshes = false, Plane? basePlane = null) => new TrackKinematics(this, target, displayMeshes, basePlane);

        class TrackKinematics : MechanismKinematics
        {
            internal TrackKinematics(Track track, Target target, bool displayMeshes, Plane? basePlane) : base(track, target, null, displayMeshes, basePlane) { }

            protected override void SetJoints(Target target, double[] prevJoints)
            {
                int externalNum = mechanism.Joints[0].Number - 6;
                if (target.External == null || target.External.Length < externalNum)
                    Errors.Add($"Track external axis not configured on this target.");
                else
                    Joints[0] = target.External[externalNum];
            }

            protected override void SetPlanes(Target target)
            {
                Planes[1] = mechanism.Joints[0].Plane;
                Planes[1].Translate(Planes[1].YAxis * Joints[0]);
            }
        }
    }
}