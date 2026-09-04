using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class NumericalKinematicsTests
{
    [Test]
    public void XmlCanSelectNumericalSolver()
    {
        var system = AbbWithSolver(nameof(NumericalKinematics));

        Assert.That(system.MechanicalGroups[0].Robot.Solver, Is.TypeOf<NumericalKinematics>());
    }

    [Test]
    public void NumericalFactoryKeepsFrankaSettings()
    {
        var robot = ((SystemFranka)TestRobots.FrankaPanda()).Robot;
        var solver = RobotKinematics.Create(typeof(NumericalKinematics), robot);

        Assert.Multiple(() =>
        {
            Assert.That(solver, Is.TypeOf<NumericalKinematics>());
            Assert.That(solver.RedundantJointIndex, Is.EqualTo(2));
            Assert.That(solver.RequiresContinuation, Is.True);
        });
    }

    [TestCase("UnknownKinematics", "Kinematics solver 'UnknownKinematics' was not found in the Robots assembly.")]
    [TestCase(nameof(RapidPostProcessor), "Kinematics solver 'RapidPostProcessor' must be a concrete RobotKinematics implementation.")]
    public void XmlRejectsInvalidSolver(string solver, string message)
    {
        var exception = Assert.Throws<ArgumentException>(() => AbbWithSolver(solver));

        Assert.That(exception!.Message, Is.EqualTo(message));
    }

    [Test]
    public void XmlRejectsIncompatibleSolverGeometry()
    {
        var exception = Assert.Throws<ArgumentException>(() => AbbWithSolver(nameof(FixedRedundancyKinematics)));

        Assert.Multiple(() =>
        {
            Assert.That(exception!.Message, Does.Contain("does not support robot geometry 'ABB.IRB120'"));
            Assert.That(exception.ParamName, Is.EqualTo("solverType"));
        });
    }

    [Test]
    public void NumericalIkSupportsCartesianJointMotion()
    {
        var robot = TestRobots.AbbNumerical();
        double[] startJoints = [0.05, 1.25, 0.05, 0.05, 0.05, 0.05];
        double[] endJoints = [0.25, 1.35, 0.15, 0.2, 0.15, 0.1];
        JointTarget endJointTarget = new(endJoints);
        var endPlane = robot.Kinematics([endJointTarget])[0].Planes[^1];

        JointTarget startTarget = new(startJoints);
        CartesianTarget endTarget = new(endPlane, motion: Motions.Joint);
        var program = new Program("NumericalJointMove", robot, [new SimpleToolpath(startTarget, endTarget)]);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(program.Code, Is.Not.Null);
        Assert.That(program.Targets[1].Joints, Is.EqualTo(endJoints).Within(1e-3));
    }

    [Test]
    public void NumericalIkDoesNotClaimARequestedConfiguration()
    {
        var system = TestRobots.AbbNumerical();
        var robot = ((IndustrialSystem)system).MechanicalGroups[0].Robot;
        double[] joints = [0.25, 1.35, 0.15, 0.2, 0.15, 0.1];
        var forward = robot.Kinematics(new JointTarget(joints));
        var target = new CartesianTarget(
            forward.Planes[^1],
            RobotConfigurations.Wrist,
            Motions.Joint);
        var solution = robot.Kinematics(target, joints);

        Assert.Multiple(() =>
        {
            Assert.That(solution.Configuration, Is.EqualTo(RobotConfigurations.None));
            Assert.That(solution.Errors, Does.Contain("Target configuration is not available."));
            Assert.That(solution.Joints, Is.EqualTo(joints).Within(1e-3));
        });
    }

    [TestCase(0)]
    [TestCase(-1e-6)]
    [TestCase(1e-6)]
    public void HalfTurnsReachTheRequestedPose(double offset)
    {
        var robot = ((IndustrialSystem)TestRobots.AbbNumerical()).MechanicalGroups[0].Robot;
        double halfPi = Math.PI / 2;
        double[] previous = [0, halfPi, halfPi, 0, halfPi, 0];
        double[] destination = [.. previous];
        destination[5] = Math.PI + offset;
        var expected = robot.Kinematics(new JointTarget(destination)).Planes[^1];
        var solution = robot.Kinematics(new CartesianTarget(expected), previous);
        var actual = solution.Planes[^1];

        Assert.Multiple(() =>
        {
            Assert.That(solution.Errors, Is.Empty);
            Assert.That(actual.Origin.DistanceTo(expected.Origin), Is.LessThan(1e-5));
            Assert.That((actual.XAxis - expected.XAxis).Length, Is.LessThan(1e-8));
            Assert.That((actual.YAxis - expected.YAxis).Length, Is.LessThan(1e-8));
        });
    }

    static IndustrialSystem AbbWithSolver(string solver)
    {
        string xml = TestRobots.AbbIrb120Xml.Replace(
            "<RobotArm ",
            $"<RobotArm solver=\"{solver}\" ",
            StringComparison.Ordinal);
        return (IndustrialSystem)FileIO.ParseRobotSystem(xml, Plane.WorldXY);
    }
}
