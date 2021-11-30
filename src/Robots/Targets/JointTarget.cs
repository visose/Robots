using System.Collections.Generic;
using System.Linq;
using static System.Math;

namespace Robots
{
    public class JointTarget : Target
    {
        public double[] Joints { get; set; }

        public JointTarget(double[] joints, Tool? tool = null, Speed? speed = null, Zone? zone = null, Command? command = null, Frame? frame = null, IEnumerable<double>? external = null) : base(tool, speed, zone, command, frame, external)
        {
            Joints = joints;
        }

        public JointTarget(double[] joints, Target target, IEnumerable<double>? external = null) : this(joints, target.Tool, target.Speed, target.Zone, target.Command, target.Frame, external ?? target.External) { }

        public static double[] Lerp(double[] a, double[] b, double t, double min, double max)
        {
            t = (t - min) / (max - min);
            if (double.IsNaN(t)) t = 0;
            var result = new double[a.Length];

            for (int i = 0; i < a.Length; i++)
            {
                result[i] = (a[i] * (1.0 - t) + b[i] * t);
            }

            return result;
        }

        public static double GetAbsoluteJoint(double joint)
        {
            double PI2 = PI * 2;
            double absJoint = Abs(joint);
            double result = absJoint - Floor(absJoint / PI2) * PI2;
            if (result > PI) result -= PI2;
            result *= Sign(joint);
            return result;
        }

        public static double[] GetAbsoluteJoints(double[] joints, double[] prevJoints)
        {
            double[] closestJoints = new double[joints.Length];

            for (int i = 0; i < joints.Length; i++)
            {
                double PI2 = PI * 2;
                double prevJoint = GetAbsoluteJoint(prevJoints[i]);
                double joint = GetAbsoluteJoint(joints[i]);
                double difference = joint - prevJoint;
                double absDifference = Abs(difference);
                if (absDifference > PI) difference = (absDifference - PI2) * Sign(difference);
                closestJoints[i] = prevJoints[i] + difference;
            }

            return closestJoints;
        }

        public override string ToString()
        {
            string type = $"Joint ({string.Join(",", Joints.Select(x => $"{x:0.###}"))})";
            string tool = $", {Tool}";
            string speed = $", {Speed}";
            string zone = $", {Zone}";
            string commands = Command != null ? ", Contains commands" : "";
            string external = External.Length > 0 ? $", {External.Length.ToString():0} external axes" : "";
            return $"Target ({type}{tool}{speed}{zone}{commands}{external})";
        }
    }
}