using static System.Math;
using static Robots.Util;

namespace Robots;

public class JointTarget : Target
{
    public double[] Joints { get; init => field = ValidateJoints(value); }

    public JointTarget(double[] joints, Tool? tool = null, Speed? speed = null, Zone? zone = null, Command? command = null, Frame? frame = null, double[]? external = null, string[]? externalCustom = null)
        : base(tool, speed, zone, command, frame, external, externalCustom)
    {
        Joints = joints;
    }

    static double[] ValidateJoints(double[] joints)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(joints.Length, 6, nameof(joints));
        ArgumentOutOfRangeException.ThrowIfGreaterThan(joints.Length, 7, nameof(joints));

        for (int i = 0; i < joints.Length; i++)
            _ = CheckFinite(joints[i], nameof(joints), $"Joint value {i} must be finite.");

        return joints;
    }

    public JointTarget(double[] joints, Target target, double[]? external = null)
        : this(joints, target.Tool, target.Speed, target.Zone, target.Command, target.Frame, external ?? target.External, target.ExternalCustom) { }

    protected override Target WithProperties(Tool tool, Speed speed, Zone zone, Command command, Frame frame, double[] external, string[]? externalCustom) =>
        new JointTarget(Joints, tool, speed, zone, command, frame, external, externalCustom);

    public static double[] Lerp(double[] a, double[] b, double t, double min, double max) =>
        Lerp(a.AsSpan(0, a.Length), b.AsSpan(0, b.Length), t, min, max);

    internal static double[] Lerp(ReadOnlySpan<double> a, ReadOnlySpan<double> b, double t, double min, double max)
    {
        ArgumentOutOfRangeException.ThrowIfNotEqual(a.Length, b.Length, nameof(b));

        t = (t - min) / (max - min);
        if (double.IsNaN(t)) t = 0;
        var result = new double[a.Length];

        for (int i = 0; i < a.Length; i++)
        {
            result[i] = a[i] * (1.0 - t) + b[i] * t;
        }

        return result;
    }

    public static double GetAbsoluteJoint(double joint)
    {
        double absJoint = Abs(joint);
        double result = absJoint - Floor(absJoint / PI2) * PI2;
        if (result > PI) result -= PI2;
        result *= Sign(joint);
        return result;
    }

    public static double[] GetAbsoluteJoints(double[] joints, double[] prevJoints) =>
        GetAbsoluteJoints(joints.AsSpan(0, joints.Length), prevJoints.AsSpan(0, prevJoints.Length));

    internal static double[] GetAbsoluteJoints(ReadOnlySpan<double> joints, ReadOnlySpan<double> prevJoints)
    {
        double[] closestJoints = new double[joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
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
        string commands = Command != Command.Default ? ", Contains commands" : "";
        string external = External.Length > 0 ? $", {External.Length.Text()} external axes" : "";
        return $"Target ({type}{tool}{speed}{zone}{commands}{external})";
    }
}
