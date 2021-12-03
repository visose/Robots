using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

abstract class RobotKinematics : MechanismKinematics
{
    protected RobotKinematics(RobotArm robot, Target target, double[]? prevJoints = null, Plane? basePlane = null) : base(robot, target, prevJoints, basePlane) { }

    protected override void SetJoints(Target target, double[]? prevJoints)
    {
        if (target is JointTarget jointTarget)
        {
            Joints = jointTarget.Joints;
        }
        else if (target is CartesianTarget cartesianTarget)
        {
            double[] joints;
            Plane tcp = target.Tool.Tcp;
            tcp.Rotate(PI, Vector3d.ZAxis, Point3d.Origin);

            Plane targetPlane = cartesianTarget.Plane;
            targetPlane.Transform(target.Frame.Plane.ToTransform());

            var transform = Transform.PlaneToPlane(Planes[0], Plane.WorldXY) * Transform.PlaneToPlane(tcp, targetPlane);

            List<string> errors;

            if (cartesianTarget.Configuration is not null || prevJoints is null)
            {
                Configuration = cartesianTarget.Configuration ?? RobotConfigurations.None;
                joints = InverseKinematics(transform, Configuration, out errors);
            }
            else
            {
                joints = GetClosestSolution(transform, prevJoints, out var configuration, out errors, out _);
                Configuration = configuration;
            }

            if (prevJoints is not null)
                Joints = JointTarget.GetAbsoluteJoints(joints, prevJoints);
            else
                Joints = joints;

            Errors.AddRange(errors);
        }
    }

    protected override void SetPlanes(Target target)
    {
        var jointTransforms = ForwardKinematics(Joints);

        if (target is JointTarget)
        {
            _ = GetClosestSolution(jointTransforms[jointTransforms.Length - 1], Joints, out var configuration, out _, out var difference);
            Configuration = difference < AngleTol ? configuration : RobotConfigurations.Undefined;
        }

        for (int i = 0; i < 6; i++)
        {
            var plane = jointTransforms[i].ToPlane();
            plane.Rotate(PI, plane.ZAxis);
            Planes[i + 1] = plane;
        }

        /* 
         {
             Planes[7] = target.Tool.Tcp;
             Planes[7].Transform(Planes[6].ToTransform());
         }
        */
    }

    protected abstract double[] InverseKinematics(Transform transform, RobotConfigurations configuration, out List<string> errors);
    protected abstract Transform[] ForwardKinematics(double[] joints);

    double SquaredDifference(double a, double b)
    {
        double difference = Abs(a - b);
        if (difference > PI)
            difference = PI * 2 - difference;
        return difference * difference;
    }

    double[] GetClosestSolution(Transform transform, double[] prevJoints, out RobotConfigurations configuration, out List<string> errors, out double difference)
    {
        var solutions = new double[8][];
        var solutionsErrors = new List<List<string>>(8);

        for (int i = 0; i < 8; i++)
        {
            solutions[i] = InverseKinematics(transform, (RobotConfigurations)i, out var solutionErrors);
            solutions[i] = JointTarget.GetAbsoluteJoints(solutions[i], prevJoints);
            solutionsErrors.Add(solutionErrors);
        }

        int closestSolutionIndex = 0;
        double closestDifference = double.MaxValue;

        for (int i = 0; i < 8; i++)
        {
            double currentDifference = prevJoints.Zip(solutions[i], (x, y) => SquaredDifference(x, y)).Sum();

            if (currentDifference < closestDifference)
            {
                closestSolutionIndex = i;
                closestDifference = currentDifference;
            }
        }

        difference = closestDifference;
        configuration = (RobotConfigurations)closestSolutionIndex;
        errors = solutionsErrors[closestSolutionIndex];
        return solutions[closestSolutionIndex];
    }
}
