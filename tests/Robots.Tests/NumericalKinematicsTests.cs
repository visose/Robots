using NUnit.Framework;

namespace Robots.Tests;

public class NumericalKinematicsTests
{
    [Test]
    public void AbbSolverMatchesWristGeometry()
    {
        var spherical = (IndustrialSystem)TestRobots.AbbIrb120();
        var offset = (IndustrialSystem)TestRobots.AbbNumerical();

        Assert.Multiple(() =>
        {
            Assert.That(spherical.MechanicalGroups[0].Robot.Solver, Is.TypeOf<SphericalWristKinematics>());
            Assert.That(offset.MechanicalGroups[0].Robot.Solver, Is.TypeOf<NumericalKinematics>());
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
}
