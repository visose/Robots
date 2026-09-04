using Rhino.Geometry;

namespace Robots;

abstract class ConfigurationKinematics(RobotArm robot) : RobotKinematics(robot)
{
    protected sealed override InverseSolutions GetInverseSolutions(
        Transform transform,
        double[] external,
        PreviousJoints prevJoints,
        RobotConfigurations? requested)
    {
        if (requested is not null || !prevJoints.HasValue)
        {
            var configuration = requested ?? RobotConfigurations.None;
            var joints = SolveConfiguration(
                transform,
                configuration,
                external,
                prevJoints,
                out var errors);
            return new([new(joints, configuration, errors)], []);
        }

        var solutions = new InverseSolution[8];

        for (int i = 0; i < solutions.Length; i++)
        {
            var current = (RobotConfigurations)i;
            var joints = SolveConfiguration(
                transform,
                current,
                external,
                prevJoints,
                out var errors);
            solutions[i] = new(joints, current, errors);
        }

        return new(solutions, []);
    }

    protected abstract double[] SolveConfiguration(
        Transform transform,
        RobotConfigurations configuration,
        double[] external,
        PreviousJoints prevJoints,
        out IReadOnlyList<string> errors);
}
