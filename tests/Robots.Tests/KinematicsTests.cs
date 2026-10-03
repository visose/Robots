using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class KinematicsTests
{
    [TestCase(0)]
    [TestCase(1e-10)]
    [TestCase(-1e-10)]
    [TestCase(Math.PI)]
    [TestCase(-Math.PI)]
    [TestCase(Math.PI - 1e-10)]
    public void SphericalWristSingularityPreservesPoseAndReportsError(double wrist)
    {
        var robot = TestRobots.AbbIrb120();
        ((IndustrialSystem)robot).MechanicalGroups[0].Robot.Joints[4].Range = new(-Math.PI, Math.PI);
        double[] joints = [0.03844474607074855, 1.2297264014974825, 0.2658775023957144,
            0.5180895819878624, wrist, 0.926030281896717];
        var expected = robot.Kinematics([new JointTarget(joints)])[0].Planes[^1];
        var actual = robot.Kinematics([new CartesianTarget(expected)], [joints])[0];

        Assert.Multiple(() =>
        {
            Assert.That(actual.Errors, Does.Contain("Target near singularity."));
            Assert.That(actual.Joints, Has.All.Matches<double>(double.IsFinite));
            Assert.That(actual.Planes[^1].Origin.DistanceTo(expected.Origin), Is.LessThan(1e-6));
            Assert.That(GeometryMath.RotationAngle(actual.Planes[^1], expected), Is.LessThan(1e-7));
        });
    }

    [TestCase(2, 0, 0, 0)]
    [TestCase(2, 5, 20, 0)]
    [TestCase(3, 0, 0, 0)]
    [TestCase(3, 5, 20, 30)]
    public void TrackAccumulatesOffsetsAndDisplacementsOnce(int axes, double x, double y, double z)
    {
        string third = axes == 3
            ? """<Prismatic number="9" a="25" d="15" minrange="-1000" maxrange="1000" maxspeed="1000"/>"""
            : "";
        string trackXml = $"""
            <Track model="XYZ" manufacturer="ABB" payload="1000" movesRobot="true">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints>
                <Prismatic number="7" a="100" d="10" minrange="-1000" maxrange="1000" maxspeed="1000"/>
                <Prismatic number="8" a="50" d="5" minrange="-1000" maxrange="1000" maxspeed="1000"/>
                {third}
              </Joints>
            </Track>
            """;
        var xml = TestRobots.AbbIrb120Xml.Replace("<RobotArm ", trackXml + "<RobotArm ", StringComparison.Ordinal);
        var robot = FileIO.ParseRobotSystem(xml, Plane.WorldXY);
        double[] external = axes == 3 ? [x, y, z] : [x, y];
        var pose = robot.Kinematics([new JointTarget([0, 1, 1, 0, 0.5, 0], external: external)])[0];
        Point3d expected = new((axes == 3 ? 175 : 150) + x, y, (axes == 3 ? 30 : 15) + z);

        Assert.Multiple(() =>
        {
            Assert.That(pose.Errors, Is.Empty);
            Assert.That(pose.Planes[axes].Origin.DistanceTo(expected), Is.LessThan(1e-9));
            Assert.That(pose.Planes[axes + 1].Origin.DistanceTo(expected), Is.LessThan(1e-9), "Robot base must follow the track.");
        });
    }

    [Test]
    public void RobotSystemRejectsWrongJointCountAtPublicBoundary()
    {
        var robot = TestRobots.AbbIrb120();
        JointTarget target = new(new double[5]);

        var exception = Assert.Throws<ArgumentException>(() => robot.Kinematics([target]));

        Assert.That(exception!.Message, Does.Contain("5 joint value(s) supplied, but 6 are required"));
    }

    [Test]
    public void RobotArmRejectsWrongJointCountAtPublicBoundary()
    {
        var robot = ((IndustrialSystem)TestRobots.AbbIrb120()).MechanicalGroups[0].Robot;
        JointTarget target = new(new double[5]);

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
        JointTarget target = new(new double[7], external: [0]);

        var exception = Assert.Throws<ArgumentException>(() => robot.Kinematics([target]));

        Assert.That(exception!.Message, Does.Contain("Redundant robots with external mechanisms are not supported"));
    }

    [Test]
    public void CustomExternalKinematicsPreservesBasePlane()
    {
        var robot = TestRobots.AbbIrb120WithCustomExternal();
        JointTarget target = new(new double[6], external: [25]);
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
        JointTarget target = new(new double[6], external: [25]);
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
        JointTarget group1Target = new(new double[6], external: [0]);
        var reference = robot.Kinematics([new JointTarget(joints), group1Target]);

        Plane coupledPlane = reference[1].Planes[1];
        Plane localPlane = reference[0].Planes[^1];
        _ = localPlane.Transform(Transform.PlaneToPlane(coupledPlane, Plane.WorldXY));

        Frame coupledFrame = new(Plane.WorldXY, coupledMechanism: 0, coupledMechanicalGroup: 1);
        CartesianTarget coupledTarget = new(localPlane, reference[0].Configuration, frame: coupledFrame);
        var solution = robot.Kinematics([coupledTarget, group1Target])[0];

        Assert.That(solution.Errors, Is.Empty);
        Assert.That(solution.Planes[^1].Origin.DistanceTo(reference[0].Planes[^1].Origin), Is.LessThan(1e-9));
    }
}
