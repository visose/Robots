using NUnit.Framework;
using Rhino.Geometry;
using Robots.Commands;

namespace Robots.Tests;

public class MotionPlannerTests
{
    [TestCase(-0.2)]
    [TestCase(-0.13)]
    public void PureRotationChecksIntermediateSingularities(double wrist)
    {
        var robot = TestRobots.AbbIrb120();
        Tool tool = new(Plane.WorldXY.WithOrigin(0, 0, -72));
        JointTarget start = new([0.3, 1.1, 0.4, -0.5, 0.2, 0.6], tool: tool);
        JointTarget endJoints = new([0.3, 1.1, 0.4, -0.5, wrist, 0.6], tool: tool);
        var endPlane = robot.Kinematics([endJoints])[0].Planes[^1];
        CartesianTarget end = new(endPlane, motion: Motions.Linear, tool: tool);

        Program program = new("Rotation", robot, [TestRobots.Toolpath(start, end)]);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Has.Some.Contains("singularity"));
            Assert.That(program.Code, Is.Null);
        });
    }

    [Test]
    public void LinearMotionChecksBranchChangesBetweenTranslationSamples()
    {
        CartesianTarget start = new(Plane.WorldZX.WithOrigin(500, 250, 600), RobotConfigurations.Wrist, Motions.Joint);
        CartesianTarget end = new(Plane.WorldZX.WithOrigin(500, 300, 600), motion: Motions.Linear);

        Program program = new("Wrist", TestRobots.UR10(), [TestRobots.Toolpath(start, end)], stepSize: 50);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Has.Some.Contains("singularity"));
            Assert.That(program.Code, Is.Null);
        });
    }

    [Test]
    public void RotationTimeUsesTheFullRelativeAngle()
    {
        var robot = TestRobots.AbbIrb120();
        Speed speed = new(rotationSpeed: 0.01);
        JointTarget start = new([0.3, 1.1, 0.4, -0.5, 0.7, 0.6], speed: speed);
        var startPlane = robot.Kinematics([start])[0].Planes[^1];
        var endPlane = startPlane;
        _ = endPlane.Rotate(0.2, startPlane.XAxis + startPlane.YAxis + startPlane.ZAxis, startPlane.Origin);
        CartesianTarget end = new(endPlane, motion: Motions.Linear, speed: speed);

        Program program = new("Rotation", robot, [TestRobots.Toolpath(start, end)]);
        program.Animate(10, false);
        var halfway = program.CurrentSimulationPose.GetLastPlane(0);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(20).Within(1e-10));
            Assert.That(GeometryMath.RotationAngle(startPlane, halfway), Is.EqualTo(0.1).Within(1e-10));
        });
    }

    [TestCase(0, false)]
    [TestCase(0, true)]
    [TestCase(1, false)]
    [TestCase(1, true)]
    public void WaitTimingRespectsItsTargetAndPhase(int target, bool runBefore)
    {
        var robot = TestRobots.AbbIrb120();
        double[][] joints = [[0.3, 1.1, 0.4, -0.5, 0.2, 0.6], [0.4, 1.1, 0.4, -0.5, 0.2, 0.6]];
        var targets = joints.Select((values, index) => new JointTarget(
            values, speed: new(time: 2), command: index == target ? new Wait(5, runBefore) : null)).ToArray();

        Program program = new("Waits", robot, [new SimpleToolpath(targets)]);
        bool waitFirst = target == 0 || runBefore;
        program.Animate(waitFirst ? 2.5 : 4.5, false);
        double held = program.CurrentSimulationPose.Kinematics[0].Joints[0];
        program.Animate(waitFirst ? 6 : 1, false);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(7).Within(1e-12));
            Assert.That(program.Targets[1].DeltaTime, Is.EqualTo(2).Within(1e-12));
            Assert.That(held, Is.EqualTo(waitFirst ? 0.3 : 0.4).Within(1e-12));
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints[0], Is.EqualTo(0.35).Within(1e-12));
        });
    }

    [TestCase(0, false)]
    [TestCase(1, true)]
    [TestCase(1, false)]
    [TestCase(3, true)]
    [TestCase(3, false)]
    public void FlyByReconstructionPreservesWaits(int target, bool runBefore)
    {
        var robot = TestRobots.AbbIrb120();
        var targets = Enumerable.Range(0, 4).Select(index => new JointTarget(
            [0.3 + index * 0.1, 1.1, 0.4, -0.5, 0.2, 0.6],
            speed: new(time: 2), zone: new(1), command: index == target ? new Wait(5, runBefore) : null)).ToArray();

        Program program = new("Waits", robot, [new SimpleToolpath(targets)]);
        double waitStart = 2 * Math.Max(0, target - (runBefore ? 1 : 0));
        program.Animate(waitStart + 2.5, false);
        double expected = 0.3 + Math.Max(0, target - (runBefore ? 1 : 0)) * 0.1;

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(11).Within(1e-12));
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints[0], Is.EqualTo(expected).Within(1e-12));
            Assert.That(program.MotionSegments.Any(segment => segment.Corner is not null), Is.True);
        });
    }

    [Test]
    public void SingleTargetWaitHasAStationarySimulation()
    {
        JointTarget target = new([0.3, 1.1, 0.4, -0.5, 0.2, 0.6], command: new Wait(5));
        Program program = new("Waits", TestRobots.AbbIrb120(), [target]);
        program.Animate(0.5);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(5));
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints, Is.EqualTo(target.Joints));
        });
    }
}
