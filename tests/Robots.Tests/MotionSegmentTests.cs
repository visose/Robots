using NUnit.Framework;
using Rhino.Geometry;
using static System.Math;

namespace Robots.Tests;

class MotionSegmentTests
{
    [TestCase(false)]
    [TestCase(true)]
    public void FlybyInMovingFrameClosesAtBothBoundaries(bool movesRobot)
    {
        string trackXml = $"""
            <Track model="Slide" manufacturer="ABB" payload="1000" movesRobot="{movesRobot.ToString().ToLowerInvariant()}">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints><Prismatic number="7" a="0" d="0" minrange="-1000" maxrange="1000" maxspeed="1000"/></Joints>
            </Track>
            """;
        var xml = TestRobots.AbbIrb120Xml.Replace("<RobotArm ", trackXml + "<RobotArm ", StringComparison.Ordinal);
        var robot = FileIO.ParseRobotSystem(xml, Plane.WorldXY);
        Frame frame = new(Plane.WorldXY, coupledMechanism: 0, coupledMechanicalGroup: 0);
        JointTarget first = new([0.3, 1.1, 0.4, -0.5, 0.7, 0.6], frame: frame, external: [0]);
        var local = robot.Kinematics([first])[0].Planes[^1];
        CartesianTarget corner = new(local, motion: Motions.Linear, frame: frame, external: [10], zone: new(2));
        CartesianTarget last = new(local, motion: Motions.Linear, frame: frame, external: [20]);
        Program program = new("MovingFrame", robot, [TestRobots.Toolpath(first, corner, last)], stepSize: 10);

        Assert.That(program.Errors, Is.Empty);
        var segment = program.MotionSegments.Single(segment => segment.Corner is not null);

        foreach (var boundary in new[] { segment.Start, segment.End })
        {
            var targets = segment.Lerp(robot, boundary.TotalTime);
            var actual = robot.Kinematics(targets, boundary.JointSets())[0];
            var expected = boundary.ProgramTargets[0].WorldPlane;

            Assert.Multiple(() =>
            {
                Assert.That(actual.Errors, Is.Empty);
                Assert.That(actual.Planes[^1].Origin.DistanceTo(expected.Origin), Is.LessThan(1e-6));
                Assert.That(GeometryMath.RotationAngle(actual.Planes[^1], expected), Is.LessThan(1e-7));
            });
        }

        program.Animate((segment.Start.TotalTime + segment.End.TotalTime) / 2, isNormalized: false);
        var middle = program.CurrentSimulationPose.Kinematics[0];
        var middleLocal = middle.Planes[^1];
        var middleFrame = middle.Planes[1];
        middleLocal.InverseOrient(ref middleFrame);
        Assert.That(middleLocal.Origin.DistanceTo(local.Origin), Is.LessThan(1e-6));
    }

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
