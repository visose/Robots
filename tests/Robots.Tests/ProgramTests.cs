using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class ProgramTests
{
    [TestCaseSource(nameof(SamplePrograms))]
    public void ProgramHasExpectedDurationAndJoints(SampleProgram sample)
    {
        var program = sample.CreateProgram();

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(sample.Duration).Within(1e-14));
            Assert.That(program.Targets[1].Joints, Is.EqualTo(sample.Joints).Within(1e-14));
        });
    }

    [TestCaseSource(nameof(SamplePrograms))]
    public void ProgramEndsAtRequestedPlane(SampleProgram sample)
    {
        var program = sample.CreateProgram();
        var plane = program.Targets[1].Planes[^1];

        Assert.Multiple(() =>
        {
            Assert.That(plane.Origin.DistanceTo(sample.EndPlane.Origin), Is.LessThan(1e-9));
            Assert.That(Vector3d.VectorAngle(plane.XAxis, sample.EndPlane.XAxis), Is.LessThan(1e-12));
            Assert.That(Vector3d.VectorAngle(plane.YAxis, sample.EndPlane.YAxis), Is.LessThan(1e-12));
        });
    }

    [Test]
    public void ProgramReportsEmptyToolpaths()
    {
        var program = new Program("P", TestRobots.AbbIrb120(), [new SimpleToolpath()]);

        Assert.Multiple(() =>
        {
            Assert.That(program.Code, Is.Null);
            Assert.That(program.Errors, Has.One.EqualTo("The program must contain at least one target."));
            Assert.That(program.Issues, Has.One.Matches<ProgramIssue>(issue =>
                issue.Level == IssueLevel.Error
                && issue.Kind == IssueKind.ToolpathInvalid));
        });
    }

    [Test]
    public void ProgramReportsMismatchedToolpathCounts()
    {
        var program = new Program(
            "P",
            TestRobots.AbbTwoGroupWithCustomExternal(),
            [TestRobots.Toolpath(new JointTarget(new double[6]))]);

        Assert.That(program.Errors, Has.One.EqualTo("You supplied 1 toolpath(s), this robot system requires 2 toolpath(s)."));
    }

    [Test]
    public void ProgramReportsInvalidFrameCoupling()
    {
        var frame = new Frame(Plane.WorldXY, coupledMechanism: 0, coupledMechanicalGroup: 1);
        var target = new JointTarget(new double[6], frame: frame);

        var program = new Program("P", TestRobots.AbbIrb120(), [TestRobots.Toolpath(target)]);

        Assert.Multiple(() =>
        {
            Assert.That(program.Code, Is.Null);
            Assert.That(program.Errors, Has.One.Contains("is set to couple a nonexistent mechanical group."));
            Assert.That(program.Issues, Has.One.Matches<ProgramIssue>(issue =>
                issue.Level == IssueLevel.Error
                && issue.Kind == IssueKind.FrameCouplingInvalid
                && issue.TargetIndex == 0
                && issue.RobotGroup == 0));
        });
    }

    [Test]
    public void ProgramUsesSpeedTimeForDuration()
    {
        var robot = TestRobots.UR10();
        Plane planeA = Plane.WorldZX.WithOrigin(200, 100, 600);
        Plane planeB = Plane.WorldZX.WithOrigin(700, 250, 600);
        CartesianTarget start = new(planeA, RobotConfigurations.Wrist, Motions.Joint);
        CartesianTarget end = new(planeB, motion: Motions.Linear, speed: new(time: 2));

        Program program = new("P", robot, [TestRobots.Toolpath(start, end)]);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(2).Within(1e-12));
            Assert.That(program.Targets[1].DeltaTime, Is.EqualTo(2).Within(1e-12));
        });
    }

    static IEnumerable<TestCaseData> SamplePrograms()
    {
        yield return new TestCaseData(Abb()).SetName("ABB sample program");
        yield return new TestCaseData(UR()).SetName("UR sample program");
    }

    static SampleProgram Abb()
    {
        var endPlane = Plane.WorldYZ;
        endPlane.Origin = new(300, -200, 610);

        return new(
            TestRobots.AbbSampleProgram,
            1.6432545251573487,
            [
                -0.72007069377409672,
                1.5806369662963811,
                -0.075569321979534809,
                -1.4960590345094886,
                0.72252891341688164,
                3.042104596978858
            ],
            endPlane);
    }

    static SampleProgram UR()
    {
        var endPlane = Plane.WorldZX;
        endPlane.Origin = new(700, 250, 600);

        return new(
            TestRobots.URSampleProgram,
            1.7425663263380393,
            [
                3.132810991378919,
                -1.2818634714414483,
                1.6947375202478607,
                2.728718604783381,
                0.008781662210846086,
                -3.141592653589793
            ],
            endPlane);
    }

    public sealed record SampleProgram(Func<Program> CreateProgram, double Duration, double[] Joints, Plane EndPlane);
}
