using NUnit.Framework;
using Rhino.Geometry;
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
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(expected));
    }

    [TestCase(Manufacturers.ABB, 6, "MoveAbsJ [[0,90,-0,0,-0,0],extj],DefaultSpeed,fine,DefaultTool;")]
    [TestCase(Manufacturers.KUKA, 6, "PTP {A1 -0,A2 -0,A3 90,A4 -0,A5 -0,A6 -0}")]
    [TestCase(Manufacturers.UR, 6, "  movej([0, 0, 0, 0, 0, 0], a=12.5664, v=0.3142, r=DefaultZone)")]
    [TestCase(Manufacturers.Staubli, 6, "movej(joints[0], DefaultTool, mdesc0000)")]
    [TestCase(Manufacturers.FrankaEmika, 7, "  motion = JointMotion([0, 0, 0, 0, 0, 0, 0])")]
    [TestCase(Manufacturers.Doosan, 6, "movej([180, -90, -90, 0, 0, -180], a=720, v=18, r=DefaultZone)")]
    [TestCase(Manufacturers.Fanuc, 6, ":J P[1] 10% FINE ;")]
    [TestCase(Manufacturers.Igus, 6, """<Joint AbortCondition="False" Nr="1" Source="Numerical" velPercent="100" acc="90" smooth="0" a1="0.000" a2="90.000" a3="90.000" a4="-0.000" a5="-0.000" a6="0.000" e1="0" e2="0" e3="0" Descr="" />""")]
    [TestCase(Manufacturers.Jaka, 6, "movj(endPosJ,0,180,5000,2.0)")]
    public void PostProcessorsGenerateRepresentativeJointMove(Manufacturers manufacturer, int jointCount, string expected)
    {
        var program = CreateProgram(manufacturer, jointCount);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(expected));
    }

    [TestCaseSource(nameof(SamplePrograms))]
    public void PostProcessorsGenerateExpectedSampleProgram(SampleProgram sample)
    {
        var program = sample.CreateProgram();
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Is.EqualTo(sample.Code));
    }

    [Test]
    public void DuplicateInvalidCommandsStayProgramErrors()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.ABB, 6);
        var command = new SetDO(2, true);
        var toolpath = TestRobots.Toolpath(
            new JointTarget(new double[6], command: command),
            new JointTarget(new double[6], command: command));

        var program = new Program("P", robot, [toolpath]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.Contains("Digital output 2: IO index is out of range."));
    }

    [Test]
    public void FrankaMultiFilePostProcessorFails()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7);
        var toolpath = TestRobots.Toolpath(
            new JointTarget(new double[7]),
            new JointTarget(new double[7]));

        var program = new Program("P", robot, [toolpath], multiFileIndices: [0, 1]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Multi-file programs are not supported on Franka Emika robots."));
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
        var toolpath = TestRobots.Toolpath(start, target);

        var program = new Program("P", robot, [toolpath]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain("data = MotionData(dynamic_rel)\n  motion = WaypointMotion(["));
        Assert.That(code, Does.Contain("robot.move(DefaultTool, motion, data)"));
    }

    [Test]
    public void WaitOnlyTargetDoesNotCorruptAxisSpeed()
    {
        var robot = TestRobots.UR10();
        var toolpath = TestRobots.Toolpath(
            new JointTarget(new double[6]),
            new JointTarget(new double[6], command: new Wait(1.0)));

        var program = new Program("P", robot, [toolpath]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(program.Duration, Is.EqualTo(1.0).Within(1e-14));
        Assert.That(program.Targets[1].DeltaTime, Is.Zero.Within(1e-14));
        Assert.That(code, Does.Contain("sleep(Wait000)"));
    }

    [Test]
    public void FanucTargetCommandsAreNotPrefixedTwice()
    {
        var program = CreateProgram(Manufacturers.Fanuc, 6, new SetDO(0, true));
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(":DO[DO1]=ON ;"));
        Assert.That(code, Does.Not.Contain("::DO"));
    }

    [Test]
    public void FanucCustomTargetCommandsGetOnePrefix()
    {
        var program = CreateProgram(Manufacturers.Fanuc, 6, new Commands.Custom(command: "CALL CLEANUP ;"));
        var code = TestRobots.FlattenCode(program);

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
        var code = TestRobots.FlattenCode(program);

        int beforeIndex = code.IndexOf(":CALL BEFORE ;", StringComparison.Ordinal);
        int motionIndex = code.IndexOf(":J P[1] 10% FINE ;", StringComparison.Ordinal);
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
        var toolpath = TestRobots.Toolpath(
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
        var toolpath = TestRobots.Toolpath(
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
        var toolpath = TestRobots.Toolpath(new JointTarget(new double[6]));

        var program = new Program("Program1", robot, [toolpath]);

        Assert.That("Program1_T_ROB1".Length, Is.EqualTo(15));
        Assert.That(program.Errors, Is.Empty);
    }

    [Test]
    public void KukaExternalAxisSpeedUsesExternalVelocity()
    {
        var robot = TestRobots.KukaWithCustomExternal();
        var toolpath = TestRobots.Toolpath(
            new JointTarget(new double[6], external: [0]),
            new JointTarget(new double[6], external: [100]));

        var program = new Program("P", robot, [toolpath]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain("$VEL_EXTAX[1] = "));
        Assert.That(code, Does.Not.Contain("$VEL_AXIS[0]"));
    }

    [Test]
    public void AbbExternalAxisHelperCanLeaveControllerValueUnspecified()
    {
        JointTarget target = new([0, 0, 0, 0, 0, 0], external: [0], externalCustom: ExternalAxes.AbbUnspecifiedAxes(1));
        Program program = new("P", TestRobots.AbbIrb120WithCustomExternal(), [TestRobots.Toolpath(target)]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain("[9E9,9E9,9E9,9E9,9E9,9E9]"));
    }

    [Test]
    public void FanucCartesianJointMoveUsesPercentSpeed()
    {
        var robot = TestRobots.FanucLrMate();
        JointTarget start = new([0.1, 1.1, 0.2, 0.15, 0.4, 0.2]);
        var plane = robot.Kinematics([start])[0].Planes[^1];
        CartesianTarget target = new(plane, motion: Motions.Joint);

        Program program = new("P", robot, [TestRobots.Toolpath(start, target)]);
        Assert.That(program.Errors, Is.Empty);

        var code = TestRobots.FlattenCode(program);

        Assert.That(code, Does.Contain(":J P[2] 10% FINE ;"));
    }

    [Test]
    public void FanucCustomToolUsesToolNumber()
    {
        Tool tool = new(Plane.WorldXY.WithOrigin(10, 0, 0), "CustomTool");
        JointTarget target = new([0, 0, 0, 0, 0, 0], tool: tool);
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Fanuc, 6);

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(": UTOOL_NUM=1 ;"));
        Assert.That(code, Does.Contain(": ! Tool 1 CustomTool TCP ;"));
        Assert.That(code, Does.Contain("    UF : 0, UT : 1,"));
    }

    [Test]
    public void FanucCustomFrameUsesFrameNumber()
    {
        Frame frame = new(Plane.WorldXY.WithOrigin(10, 0, 0), name: "CustomFrame");
        JointTarget target = new([0, 0, 0, 0, 0, 0], frame: frame);
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Fanuc, 6);

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain(": UFRAME_NUM=1 ;"));
        Assert.That(code, Does.Contain(": ! Frame 1 CustomFrame ;"));
        Assert.That(code, Does.Contain("    UF : 1, UT : 1,"));
    }

    [Test]
    public void FanucControllerNumberedToolAndFrameUseConfiguredNumbers()
    {
        Tool tool = new(Plane.WorldXY, "ControllerTool", number: 7);
        Frame frame = new(Plane.WorldXY, name: "ControllerFrame", number: 3);
        JointTarget target = new([0, 0, 0, 0, 0, 0], tool: tool, frame: frame);
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Fanuc, 6);

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain("    UF : 3, UT : 7,"));
        Assert.That(code, Does.Not.Contain("ControllerTool TCP"));
        Assert.That(code, Does.Not.Contain("ControllerFrame"));
    }

    [Test]
    public void FanucControllerToolWithoutNumberFails()
    {
        Tool tool = new(Plane.WorldXY, "ControllerTool", useController: true);
        JointTarget target = new([0, 0, 0, 0, 0, 0], tool: tool);
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Fanuc, 6);

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Fanuc controller tools require a tool number."));
    }

    [Test]
    public void FanucControllerFrameWithoutNumberFails()
    {
        Frame frame = new(Plane.WorldXY, name: "ControllerFrame", useController: true);
        JointTarget target = new([0, 0, 0, 0, 0, 0], frame: frame);
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Fanuc, 6);

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Fanuc controller frames require a frame number."));
    }

    [Test]
    public void UrProcessMotionGeneratesMoveP()
    {
        var robot = TestRobots.UR10();
        var planeA = Plane.WorldZX;
        var planeB = Plane.WorldZX;
        planeA.Origin = new(200, 100, 600);
        planeB.Origin = new(700, 250, 600);
        var toolpath = TestRobots.Toolpath(
            new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint),
            new CartesianTarget(planeB, motion: Motions.Process));

        var program = new Program("P", robot, [toolpath]);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(code, Does.Contain("  movep(p["));
        Assert.That(code, Does.Not.Contain("  movel(p["));
    }

    [Test]
    public void UrProcessMotionWithTimeFails()
    {
        var speed = new Speed(time: 2);
        var program = CreateProcessProgram(TestRobots.UR10(), speed);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Process motion does not support time-based speed on UR robots; target 1 in robot 0 uses Speed.Time."));
    }

    [Test]
    public void ProcessMotionFailsOnNonUrRobots()
    {
        var program = CreateProcessProgram(TestRobots.AbbIrb120());

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Process motion is only supported on UR robots; target 1 in robot 0 is unsupported."));
    }

    static Program CreateProcessProgram(RobotSystem robot, Speed? speed = null)
    {
        var start = new JointTarget(new double[6]);
        var endPlane = robot.Kinematics([start])[0].Planes[^1];
        var processTarget = new CartesianTarget(endPlane, motion: Motions.Process, speed: speed);

        return new("P", robot, [TestRobots.Toolpath(start, processTarget)]);
    }

    static Program CreateProgram(Manufacturers manufacturer, int jointCount, Command command)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var target = new JointTarget(new double[jointCount], command: command);
        return new("P", robot, [TestRobots.Toolpath(target)]);
    }

    static Program CreateProgram(Manufacturers manufacturer, int jointCount)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var target = new JointTarget(new double[jointCount]);
        return new("P", robot, [TestRobots.Toolpath(target)]);
    }

    static IEnumerable<TestCaseData> SamplePrograms()
    {
        yield return new TestCaseData(new SampleProgram(
            TestRobots.AbbSampleProgram,
            """
            MODULE TestProgram_T_ROB1
            VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];
            VAR confdata conf := [0,0,0,0];
            PERS tooldata DefaultTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
            TASK PERS wobjdata DefaultFrame:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
            TASK PERS speeddata DefaultSpeed:=[100,180,5000,1080];
            TASK PERS speeddata Speed000:=[300,180,5000,1080];
            PROC Main()
            ConfL \Off;
            MoveAbsJ [[41.257,-0.5638,4.3298,85.7179,-41.3979,5.7002],extj],DefaultSpeed,fine,DefaultTool;
            MoveL [[300,-200,610],[0.5,0.5,0.5,0.5],conf,extj],Speed000,fine,DefaultTool \WObj:=DefaultFrame;
            ENDPROC
            ENDMODULE
            """)).SetName("ABB sample program code");

        yield return new TestCaseData(new SampleProgram(
            TestRobots.URSampleProgram,
            """
            def Program():
              DefaultToolTcp = p[0, 0, 0, 0, 0, 1.5708]
              DefaultToolWeight = 0
              DefaultToolCog = [0, 0, 0]
              DefaultSpeed = 0.1
              Speed000 = 0.3
              DefaultZone = 0
              set_tcp(DefaultToolTcp)
              set_payload(DefaultToolWeight, DefaultToolCog)
              movej([2.2208, -2.4093, 2.5006, 3.0503, 0.9208, -3.1416], a=12.5664, v=0.3142, r=DefaultZone)
              movel(p[0.7, 0.25, 0.6, -1.2092, -1.2092, -1.2092], a=2.5, v=Speed000, r=DefaultZone)
            end
            """)).SetName("UR sample program code");
    }

    public sealed record SampleProgram(Func<Program> CreateProgram, string Code);
}
