using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class KinematicsTests
{
    [Test]
    public void RobotSystemRejectsWrongJointCountAtPublicBoundary()
    {
        var robot = TestRobots.AbbIrb120();
        var target = new JointTarget(new double[5]);

        var exception = Assert.Throws<ArgumentException>(() => robot.Kinematics([target]));

        Assert.That(exception!.Message, Does.Contain("5 joint value(s) supplied, but 6 are required"));
    }

    [Test]
    public void RobotArmRejectsWrongJointCountAtPublicBoundary()
    {
        var robot = ((IndustrialSystem)TestRobots.AbbIrb120()).MechanicalGroups[0].Robot;
        var target = new JointTarget(new double[5]);

        var exception = Assert.Throws<ArgumentException>(() => robot.Kinematics(target));

        Assert.That(exception!.Message, Does.Contain("must contain 6 value(s), but 5 were supplied"));
    }

    [Test]
    public void RobotArmRejectsInvalidAuxiliaryAxisCountAtPublicBoundary()
    {
        var sixAxis = ((SingleGroupSystem)TestRobots.UR10()).Robot;
        var redundant = ((SingleGroupSystem)TestRobots.FrankaPanda()).Robot;

        var unsupported = Assert.Throws<ArgumentException>(() =>
            sixAxis.Kinematics(new JointTarget(new double[6], external: [0])));
        var excess = Assert.Throws<ArgumentException>(() =>
            redundant.Kinematics(new JointTarget(new double[7], external: [0, 0])));

        Assert.Multiple(() =>
        {
            Assert.That(unsupported!.Message, Does.Contain("does not have external axes"));
            Assert.That(excess!.Message, Does.Contain("at most one redundant joint value is accepted"));
        });
    }

    [Test]
    public void RedundantRobotRejectsRealExternalMechanisms()
    {
        var robot = TestRobots.FrankaPandaWithCustomExternal();
        var target = new JointTarget(new double[7], external: [0]);

        var exception = Assert.Throws<ArgumentException>(() => robot.Kinematics([target]));

        Assert.That(exception!.Message, Does.Contain("Redundant robots with external mechanisms are not supported"));
    }

    [Test]
    public void CustomExternalKinematicsPreservesBasePlane()
    {
        var robot = TestRobots.AbbIrb120WithCustomExternal();
        var target = new JointTarget(new double[6], external: [25]);
        var solution = robot.Kinematics([target])[0];
        double[] expectedJoints = [0, 0, 0, 0, 0, 0, 25];

        Assert.That(solution.Errors, Is.Empty);
        Assert.That(solution.Joints, Is.EqualTo(expectedJoints).Within(1e-12));
        Assert.That(solution.Planes[0].Origin, Is.EqualTo(new Point3d(100, 20, 0)));
        Assert.That(solution.Planes[1].Origin, Is.EqualTo(new Point3d(100, 20, 0)));
    }

    [Test]
    public void SingleGroupSystemRetainsExternalMechanisms()
    {
        var system = (SystemUR)TestRobots.UR10WithCustomExternal();
        var target = new JointTarget(new double[6], external: [25]);
        var solution = system.Kinematics([target])[0];

        Assert.Multiple(() =>
        {
            Assert.That(system.MechanicalGroup, Is.SameAs(system.MechanicalGroups[0]));
            Assert.That(system.Robot, Is.SameAs(system.MechanicalGroup.Robot));
            Assert.That(system.MechanicalGroup.Externals, Has.Length.EqualTo(1));
            Assert.That(system.DefaultPose.Planes[0], Has.Length.EqualTo(9));
            Assert.That(system.DefaultPose.Meshes[0], Has.Length.EqualTo(9));
            Assert.That(solution.Joints, Has.Length.EqualTo(7));
            Assert.That(solution.Joints[^1], Is.EqualTo(25));
            Assert.That(solution.Planes, Has.Length.EqualTo(10));
            Assert.That(solution.Errors, Is.Empty);
        });
    }

    [Test]
    public void CartesianTargetCanCoupleToExternalMechanismInAnotherGroup()
    {
        var robot = TestRobots.AbbTwoGroupWithCustomExternal();
        double[] joints = [0, 0.2, -0.3, 0.1, 0.2, -0.1];
        var group1Target = new JointTarget(new double[6], external: [0]);
        var reference = robot.Kinematics([new JointTarget(joints), group1Target]);

        Plane coupledPlane = reference[1].Planes[1];
        Plane localPlane = reference[0].Planes[^1];
        _ = localPlane.Transform(Transform.PlaneToPlane(coupledPlane, Plane.WorldXY));

        var coupledFrame = new Frame(Plane.WorldXY, coupledMechanism: 0, coupledMechanicalGroup: 1);
        var coupledTarget = new CartesianTarget(localPlane, reference[0].Configuration, frame: coupledFrame);
        var solution = robot.Kinematics([coupledTarget, group1Target])[0];

        Assert.That(solution.Errors, Is.Empty);
        Assert.That(solution.Planes[^1].Origin.DistanceTo(reference[0].Planes[^1].Origin), Is.LessThan(1e-9));
    }
}
