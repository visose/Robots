using Rhino.Geometry;

namespace Robots
{
    public class Track : Mechanism
    {
        internal Track(string model, Manufacturers manufacturer, double payload, Plane basePlane, Mesh? baseMesh, Joint[] joints, bool movesRobot) : base(model, manufacturer, payload, basePlane, baseMesh, joints, movesRobot) { }

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

        public override KinematicSolution Kinematics(Target target, double[]? prevJoints = null, Plane? basePlane = null) => new TrackKinematics(this, target, basePlane);

        class TrackKinematics : MechanismKinematics
        {
            internal TrackKinematics(Track track, Target target, Plane? basePlane) : base(track, target, null, basePlane) { }

            protected override void SetJoints(Target target, double[]? prevJoints)
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