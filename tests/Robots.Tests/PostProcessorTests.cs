using NUnit.Framework;
using Robots.Commands;

namespace Robots.Tests;

public class PostProcessorTests
{
    const string MessageText = "Hello \"Robots\"\nNext";

    [TestCase(Manufacturers.ABB, 6, """TPWrite "Hello ""Robots""\0ANext";""")]
    [TestCase(Manufacturers.KUKA, 6, """Hello "Robots" Next""")]
    [TestCase(Manufacturers.UR, 6, """textmsg("Hello \u0022Robots\u0022\nNext")""")]
    [TestCase(Manufacturers.Staubli, 6, """putln("Hello \u0022Robots\u0022\nNext")""")]
    [TestCase(Manufacturers.FrankaEmika, 7, """print("Hello \u0022Robots\u0022\nNext")""")]
    [TestCase(Manufacturers.Doosan, 6, """tp_log("Hello \u0022Robots\u0022\nNext")""")]
    [TestCase(Manufacturers.Fanuc, 6, """:MESSAGE["Hello ""Robots"" Next"] ;""")]
    [TestCase(Manufacturers.Igus, 6, """Descr="Hello &quot;Robots&quot;&#xA;Next" />""")]
    [TestCase(Manufacturers.Jaka, 6, """print("Hello \u0022Robots\u0022\nNext")""")]
    public void PostProcessorsGenerateMessageCommand(Manufacturers manufacturer, int jointCount, string expected)
    {
        var program = CreateProgram(manufacturer, jointCount, new Message(MessageText));
        var code = FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(expected));
    }

    [TestCase(Manufacturers.ABB, 6, "MoveAbsJ [[0,90,-0,0,-0,0],extj],DefaultSpeed,fine,DefaultTool;")]
    [TestCase(Manufacturers.KUKA, 6, "PTP {A1 -0,A2 -0,A3 90,A4 -0,A5 -0,A6 -0}")]
    [TestCase(Manufacturers.UR, 6, "  movej([0, 0, 0, 0, 0, 0], a=3.1416, v=0.3142, r=DefaultZone)")]
    [TestCase(Manufacturers.Staubli, 6, "movej(joints[0], DefaultTool, mdesc0000)")]
    [TestCase(Manufacturers.FrankaEmika, 7, "  motion = JointMotion([0, 0, 0, 0, 0, 0, 0])")]
    [TestCase(Manufacturers.Doosan, 6, "movej([180, -90, -90, 0, 0, -180], a=180, v=18, r=DefaultZone)")]
    [TestCase(Manufacturers.Fanuc, 6, ":J P[1] 0% FINE ;")]
    [TestCase(Manufacturers.Igus, 6, """<Joint AbortCondition="False" Nr="1" Source="Numerical" velPercent="100" acc="90" smooth="0" a1="0.000" a2="90.000" a3="90.000" a4="-0.000" a5="-0.000" a6="0.000" e1="0" e2="0" e3="0" Descr="" />""")]
    [TestCase(Manufacturers.Jaka, 6, "movj(endPosJ,0,180,5000,2.0)")]
    public void PostProcessorsGenerateRepresentativeJointMove(Manufacturers manufacturer, int jointCount, string expected)
    {
        var program = CreateProgram(manufacturer, jointCount);
        var code = FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(expected));
    }

    [Test]
    public void DuplicateInvalidCommandsStayProgramErrors()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.ABB, 6);
        var command = new SetDO(2, true);
        var toolpath = Toolpath(
            new JointTarget(new double[6], command: command),
            new JointTarget(new double[6], command: command));

        var program = new Program("P", robot, [toolpath]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.Contains("Digital output 2: IO index is out of range."));
    }

    [TestCase(Manufacturers.FrankaEmika, 7, "Franka Emika")]
    [TestCase(Manufacturers.UR, 6, "UR")]
    public void UnsupportedMultiFilePostProcessorsFail(Manufacturers manufacturer, int jointCount, string robotName)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var toolpath = Toolpath(
            new JointTarget(new double[jointCount]),
            new JointTarget(new double[jointCount]));

        var program = new Program("P", robot, [toolpath], multiFileIndices: [0, 1]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo($"Multi-file programs are not supported on {robotName} robots."));
    }

    [TestCase(Manufacturers.Igus, 6, "Igus")]
    [TestCase(Manufacturers.Jaka, 6, "Jaka")]
    public void UnsupportedCommandDeclarationsFail(Manufacturers manufacturer, int jointCount, string robotName)
    {
        var command = new Commands.Custom(command: "noop()", declaration: "global int CustomCommand");
        var program = CreateProgram(manufacturer, jointCount, command);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo($"Command declarations are not implemented for {robotName} robots."));
    }

    [Test]
    public void FrankaCartesianWaypointMotionDefinesMotionData()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7);
        var start = new JointTarget([0, -0.785, 0, -2.356, 0, 1.571, 0.785]);
        var plane = robot.Kinematics([start])[0].Planes[^1];
        var target = new CartesianTarget(plane, motion: Motions.Linear);
        var toolpath = Toolpath(start, target);

        var program = new Program("P", robot, [toolpath]);
        var code = FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain("data = MotionData(dynamic_rel)\n  motion = WaypointMotion(["));
        Assert.That(code, Does.Contain("robot.move(DefaultTool, motion, data)"));
    }

    [Test]
    public void WaitOnlyTargetDoesNotCorruptAxisSpeed()
    {
        var robot = TestRobots.UR10();
        var toolpath = Toolpath(
            new JointTarget(new double[6]),
            new JointTarget(new double[6], command: new Wait(1.0)));

        var program = new Program("P", robot, [toolpath]);
        var code = FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(program.Duration, Is.EqualTo(1.0).Within(1e-14));
        Assert.That(program.Targets[1].DeltaTime, Is.Zero.Within(1e-14));
        Assert.That(code, Does.Contain("sleep(Wait000)"));
    }

    [Test]
    public void FanucTargetCommandsAreNotPrefixedTwice()
    {
        var program = CreateProgram(Manufacturers.Fanuc, 6, new SetDO(0, true));
        var code = FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(":DO[DO1]=ON ;"));
        Assert.That(code, Does.Not.Contain("::DO"));
    }

    [Test]
    public void FanucCustomTargetCommandsGetOnePrefix()
    {
        var program = CreateProgram(Manufacturers.Fanuc, 6, new Commands.Custom(command: "CALL CLEANUP ;"));
        var code = FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(":CALL CLEANUP ;"));
        Assert.That(code, Does.Not.Contain("::CALL CLEANUP ;"));
    }

    [Test]
    public void FanucTargetCommandsStayAroundMotionInOrder()
    {
        var before = new Commands.Custom(command: "CALL BEFORE ;") { RunBefore = true };
        var after = new Commands.Custom(command: "CALL AFTER ;");
        var program = CreateProgram(Manufacturers.Fanuc, 6, new Group([before, after]));
        var code = FlattenCode(program);

        int beforeIndex = code.IndexOf(":CALL BEFORE ;", StringComparison.Ordinal);
        int motionIndex = code.IndexOf(":J P[1] 0% FINE ;", StringComparison.Ordinal);
        int afterIndex = code.IndexOf(":CALL AFTER ;", StringComparison.Ordinal);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(beforeIndex, Is.GreaterThanOrEqualTo(0));
        Assert.That(motionIndex, Is.GreaterThan(beforeIndex));
        Assert.That(afterIndex, Is.GreaterThan(motionIndex));
        Assert.That(code, Does.Not.Contain("::CALL"));
    }

    [Test]
    public void UnsupportedPostProcessorFeatureKeepsPublicErrorAndIssueKind()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.UR, 6);
        var toolpath = Toolpath(
            new JointTarget(new double[6]),
            new JointTarget(new double[6]));

        var program = new Program("P", robot, [toolpath], multiFileIndices: [0, 1]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Multi-file programs are not supported on UR robots."));
        Assert.That(program.Issues, Has.One.Matches<ProgramIssue>(issue =>
            issue.Level == IssueLevel.Error &&
            issue.Kind == IssueKind.UnsupportedPostProcessorFeature &&
            issue.Message == "Multi-file programs are not supported on UR robots."));
    }

    [Test]
    public void IgusMultiFileReferencesSavedSubFileNames()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Igus, 6);
        var toolpath = Toolpath(
            new JointTarget(new double[6]),
            new JointTarget(new double[6]));

        var program = new Program("P", robot, [toolpath], multiFileIndices: [0, 1]);
        var code = program.Code ?? throw new InvalidOperationException("Program code was not generated.");
        var mainCode = string.Join("\n", code[0][0]);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(mainCode, Does.Contain("File=\"P_001.xml\""));
        Assert.That(mainCode, Does.Contain("File=\"P_002.xml\""));
    }

    [Test]
    public void StaubliAllowsFifteenCharacterProgramGroupName()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Staubli, 6);
        var toolpath = Toolpath(new JointTarget(new double[6]));

        var program = new Program("Program1", robot, [toolpath]);

        Assert.That("Program1_T_ROB1".Length, Is.EqualTo(15));
        Assert.That(program.Errors, Is.Empty);
    }

    static Program CreateProgram(Manufacturers manufacturer, int jointCount, Command command)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var target = new JointTarget(new double[jointCount], command: command);
        return new("P", robot, [Toolpath(target)]);
    }

    static Program CreateProgram(Manufacturers manufacturer, int jointCount)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var target = new JointTarget(new double[jointCount]);
        return new("P", robot, [Toolpath(target)]);
    }

    static SimpleToolpath Toolpath(params Target[] targets)
    {
        var toolpath = new SimpleToolpath();

        foreach (var target in targets)
            toolpath.Add(target);

        return toolpath;
    }

    static string FlattenCode(Program program)
    {
        var code = program.Code ?? throw new InvalidOperationException("Program code was not generated.");
        return string.Join("\n", code.SelectMany(group => group).SelectMany(file => file)).ReplaceLineEndings("\n");
    }
}
