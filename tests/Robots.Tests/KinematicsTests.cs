using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class KinematicsTests
{
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
