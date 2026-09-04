using NUnit.Framework;
using static System.Math;

namespace Robots.Tests;

class MotionSegmentTests
{
    [Test]
    public void MechanismOrderDoesNotChangeJointMetadataOrTiming()
    {
        var robot = TestRobots.AbbIrb120WithCustomExternal();
        double[] joints = [0.3, 1.1, 0.4, -0.5, 0.7, 0.6];
        Program program = new("P", robot, [TestRobots.Toolpath(
            new JointTarget(joints, external: [0]),
            new JointTarget(joints, external: [1000]))]);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(robot.GetJoints(0).Select(joint => joint.Number), Is.EqualTo(Enumerable.Range(0, 7)));
            Assert.That(program.Duration, Is.EqualTo(1).Within(1e-12));
        });
    }

    [TestCase(false, 10)]
    [TestCase(true, 20)]
    public void CollisionDivisionsUseLinearUnitsForPrismaticJoints(bool flyby, int expected)
    {
        var robot = TestRobots.AbbIrb120WithCustomExternal();
        double[] joints = [0.3, 1.1, 0.4, -0.5, 0.7, 0.6];
        Program program = new("P", robot, [TestRobots.Toolpath(
            new JointTarget(joints, external: [0]),
            new JointTarget(joints, zone: new Zone(flyby ? 10 : 0), external: [1000]),
            new JointTarget(joints, external: [0]))]);
        Assert.That(program.Errors, Is.Empty);
        MotionSegment segment = flyby
            ? new(program.Targets[0], program.Targets[2], program.Targets[1], program.Targets[2])
            : new(program.Targets[0], program.Targets[1]);

        Assert.That(segment.GetDivisions(robot, 100, PI / 4), Is.EqualTo(expected));
    }

    [Test]
    public void CollisionDivisionsRetainAngularUnitsForRevoluteJoints()
    {
        var robot = TestRobots.AbbIrb120();
        Program program = new("P", robot, [TestRobots.Toolpath(
            new JointTarget([0.3, 1.1, 0.4, -0.5, 0.7, 0.6]),
            new JointTarget([1.3, 1.1, 0.4, -0.5, 0.7, 0.6]))]);
        Assert.That(program.Errors, Is.Empty);
        MotionSegment segment = new(program.Targets[0], program.Targets[1]);

        Assert.That(segment.GetDivisions(robot, 10000, PI / 4), Is.EqualTo(2));
    }
}
