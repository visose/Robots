using System.Xml.Linq;
using NUnit.Framework;
using Rhino.Geometry;
using Robots.Commands;
using CultureInfo = System.Globalization.CultureInfo;

namespace Robots.Tests;

public class PostProcessorTests
{
    [TestCase("URScript")]
    [TestCase("DRL")]
    [TestCase("Frankx")]
    [TestCase("Franky")]
    public void SingleGroupPostProcessorsRejectExternalAxes(string dialect)
    {
        var (robot, jointCount) = dialect switch
        {
            "URScript" => (TestRobots.UR10WithCustomExternal(), 6),
            "DRL" => (TestRobots.DoosanWithCustomExternal(), 6),
            "Frankx" => (TestRobots.FrankaPandaWithCustomExternal("FrankxPostProcessor"), 7),
            "Franky" => (TestRobots.FrankaPandaWithCustomExternal("FrankyPostProcessor"), 7),
            _ => throw new ArgumentOutOfRangeException(nameof(dialect))
        };
        var target = new JointTarget(new double[jointCount], external: [0]);

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);

        Assert.That(program.Issues, Has.Some.Matches<ProgramIssue>(issue =>
            issue.Kind == IssueKind.UnsupportedPostProcessorFeature
            && issue.Message.Contains("External axes are not supported", StringComparison.Ordinal)));
    }

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
    [TestCase(Manufacturers.Igus, 6, """<Joint AbortCondition="False" Nr="1" Source="Numerical" velPercent="100" acc="90" smooth="0" a1="-0.000" a2="90.000" a3="90.000" a4="-0.000" a5="-0.000" a6="-0.000" e1="0" e2="0" e3="0" Descr="" />""")]
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

    [TestCase(Manufacturers.ABB, 6)]
    [TestCase(Manufacturers.KUKA, 6)]
    [TestCase(Manufacturers.UR, 6)]
    [TestCase(Manufacturers.Staubli, 6)]
    [TestCase(Manufacturers.FrankaEmika, 7)]
    [TestCase(Manufacturers.Doosan, 6)]
    [TestCase(Manufacturers.Fanuc, 6)]
    [TestCase(Manufacturers.Igus, 6)]
    [TestCase(Manufacturers.Jaka, 6)]
    public void PostProcessorNumbersUseInvariantCulture(Manufacturers manufacturer, int jointCount)
    {
        CultureInfo culture = CultureInfo.CurrentCulture;

        try
        {
            CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
            string expected = TestRobots.FlattenCode(CreateProgram(manufacturer, jointCount));
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");
            string actual = TestRobots.FlattenCode(CreateProgram(manufacturer, jointCount));

            Assert.Multiple(() =>
            {
                Assert.That(actual, Is.EqualTo(expected));
                Assert.That(CultureInfo.CurrentCulture, Is.EqualTo(CultureInfo.GetCultureInfo("fr-FR")));
            });
        }
        finally
        {
            CultureInfo.CurrentCulture = culture;
        }
    }

    [Test]
    public void AbbPgfUsesCrLfLineEndings()
    {
        string actual = RapidProgramFile.CreatePgf("TestProgram_T_ROB1.mod");
        string expected = """
        <?xml version="1.0" encoding="ISO-8859-1" ?>
        <Program>
            <Module>TestProgram_T_ROB1.mod</Module>
        </Program>
        """.UseCRLF();

        Assert.That(actual, Is.EqualTo(expected));
    }

    [Test]
    public void AbbOmniCoreMultiFileReferencesSavedModxFiles()
    {
        var robot = TestRobots.AbbIrb120(omniCore: true);
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(new double[6]), new JointTarget(new double[6]))],
            multiFileIndices: [0, 1]);
        var output = Path.Combine(Path.GetTempPath(), $"RobotsTests-{Guid.NewGuid():N}");

        try
        {
            Assert.That(program.Errors, Is.Empty);
            program.Save(output);
            var folder = Path.Combine(output, "P");
            var main = File.ReadAllText(Path.Combine(folder, "P_T_ROB1.modx"));

            Assert.Multiple(() =>
            {
                Assert.That(main, Does.Contain("P_T_ROB1_000.modx"));
                Assert.That(main, Does.Contain("P_T_ROB1_001.modx"));
                Assert.That(File.Exists(Path.Combine(folder, "P_T_ROB1_000.modx")), Is.True);
                Assert.That(File.Exists(Path.Combine(folder, "P_T_ROB1_001.modx")), Is.True);
            });
        }
        finally
        {
            if (Directory.Exists(output))
                Directory.Delete(output, recursive: true);
        }
    }

    [Test]
    public void ProgramSaveUsesSelectedPostProcessor()
    {
        SavingPostProcessor postProcessor = new();
        var robot = TestRobots.PostProcessorRobot(
            Manufacturers.ABB,
            6,
            postProcessorOverride: postProcessor);
        Program program = new(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(new double[6]))]);

        program.Save("output");

        Assert.Multiple(() =>
        {
            Assert.That(postProcessor.Program, Is.SameAs(program));
            Assert.That(postProcessor.Folder, Is.EqualTo("output"));
        });
    }

    [Test]
    public void KrcNameWarningBelongsToKrlPostProcessor()
    {
        const string name = "ABCDEFGHIJKLMNOPQ";
        Program krl = new(
            name,
            TestRobots.PostProcessorRobot(Manufacturers.KUKA, 6),
            [TestRobots.Toolpath(new JointTarget(new double[6]))]);
        Program alternate = new(
            name,
            TestRobots.PostProcessorRobot(
                Manufacturers.KUKA,
                6,
                postProcessorOverride: new SavingPostProcessor()),
            [TestRobots.Toolpath(new JointTarget(new double[6]))]);

        Assert.Multiple(() =>
        {
            Assert.That(krl.Warnings, Has.One.Contains("older KRC2 or KRC3 controller"));
            Assert.That(alternate.Warnings, Has.None.Contains("older KRC2 or KRC3 controller"));
        });
    }

    [TestCase(Manufacturers.KUKA, 6)]
    [TestCase(Manufacturers.Staubli, 6)]
    [TestCase(Manufacturers.FrankaEmika, 7)]
    public void ProgramCodeContainsOnlySingleLines(Manufacturers manufacturer, int jointCount)
    {
        var program = CreateProgram(manufacturer, jointCount);
        var code = program.Code ?? throw new InvalidOperationException("Program code was not generated.");
        var lines = code.SelectMany(group => group).SelectMany(file => file);

        Assert.That(program.Errors, Is.Empty);
        Assert.That(lines, Has.None.Matches<string>(line => line.Contains('\r') || line.Contains('\n')));
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
        Assert.That(code, Does.Contain("""
            data = MotionData(dynamic_rel)
              motion = WaypointMotion([
            """.UseLF()));
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
    public void IgusCommandsUseMappedOutputsAndSequentialNumbers()
    {
        const string io = """<IO><DO names="21"/><DI names="21"/><AO names="1"/><AI names="1"/></IO>""";
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Igus, 6, io: io);
        var before = new SetDO(0, true, runBefore: true);
        var after = new PulseDO(0);
        var target = new JointTarget(new double[6], command: new Group([before, after]));
        var program = new Program("P", robot, [TestRobots.Toolpath(target)]);
        var code = program.Code ?? throw new InvalidOperationException("Program code was not generated.");
        var numbered = code[0][0]
            .Where(line => line.Contains(" Nr=\"", StringComparison.Ordinal))
            .ToArray();

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(numbered, Has.Length.EqualTo(5));
            Assert.That(numbered[0], Is.EqualTo("<Output Nr=\"1\" Channel=\"DOut21\" State=\"True\" />"));
            Assert.That(numbered[1], Does.StartWith("<Joint ").And.Contain("Nr=\"2\""));
            Assert.That(numbered[2], Is.EqualTo("<Output Nr=\"3\" Channel=\"DOut21\" State=\"True\" />"));
            Assert.That(numbered[3], Is.EqualTo("<Wait Nr=\"4\" Type=\"Time\" Seconds=\"0.2\" />"));
            Assert.That(numbered[4], Is.EqualTo("<Output Nr=\"5\" Channel=\"DOut21\" State=\"False\" />"));
        });
    }

    [Test]
    public void IgusJointOutputUsesControllerAngles()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Igus, 6);
        JointTarget target = new([Math.PI / 6, Math.PI / 3, Math.PI / 4, -Math.PI / 6, -Math.PI / 4, Math.PI / 2]);
        Program program = new("P", robot, [TestRobots.Toolpath(target)]);
        Assert.That(program.Errors, Is.Empty);

        var move = XDocument.Parse(TestRobots.FlattenCode(program)).Descendants("Joint").Single();
        var angles = Enumerable.Range(1, 6).Select(i => (double)move.Attribute($"a{i}")!).ToArray();
        double[] expected = [-30, 30, 45, 30, 45, -90];
        Assert.That(angles, Is.EqualTo(expected));
    }

    [Test]
    public void FormatterCanHandleCommandWithoutOutput()
    {
        var formatter = new SuppressingFormatter();
        var system = TestRobots.AbbIrb120();

        bool handled = formatter.TryGetCommand(new Stop(), system, Target.Default, out string code);

        Assert.Multiple(() =>
        {
            Assert.That(handled, Is.True);
            Assert.That(code, Is.Empty);
        });
    }

    [TestCase(Manufacturers.All, true)]
    [TestCase(Manufacturers.ABB, true)]
    [TestCase(Manufacturers.KUKA, false)]
    public void DeclarationOnlyCommandsRequireMatchingManufacturer(Manufacturers manufacturer, bool supported)
    {
        const string declaration = "VAR num userValue := 1;";
        var command = new Commands.Custom(manufacturer: manufacturer, declaration: declaration);
        var program = CreateProgram(Manufacturers.ABB, 6, command);

        if (supported)
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(TestRobots.FlattenCode(program), Does.Contain(declaration));
        }
        else
        {
            Assert.That(program.Code, Is.Null);
            Assert.That(program.Errors, Has.One.Contains("Command CustomCommand is not implemented"));
        }
    }

    [TestCase(false)]
    [TestCase(true)]
    public void DoosanDeclaresInitialCommandValuesBeforeUse(bool multiFile)
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.Doosan, 6);
        var targets = TestRobots.Toolpath(new JointTarget(new double[6]), new JointTarget(new double[6]));
        Group init = new([new Wait(1), new SetAO(0, 0.5)]);
        Program program = new("P", robot, [targets], init, multiFile ? [0, 1] : null);
        var code = TestRobots.FlattenCode(program);

        Assert.That(program.Errors, Is.Empty);

        foreach (var (declaration, command) in new[] { ("Wait000 = 1", "wait(Wait000)"), ("SetAO000 = 5", "val=SetAO000") })
        {
            int declared = code.IndexOf(declaration, StringComparison.Ordinal);
            int used = code.IndexOf(command, StringComparison.Ordinal);
            Assert.That(declared, Is.GreaterThanOrEqualTo(0));
            Assert.That(used, Is.GreaterThan(declared));
        }
    }

    [TestCase(Manufacturers.Jaka, Motions.Joint)]
    [TestCase(Manufacturers.Jaka, Motions.Linear)]
    [TestCase(Manufacturers.Igus, Motions.Joint)]
    [TestCase(Manufacturers.Igus, Motions.Linear)]
    public void CartesianCodeIsIndependentOfWorldPlacement(Manufacturers manufacturer, Motions motion)
    {
        var robot = TestRobots.SphericalRobot(manufacturer);
        JointTarget start = new([0.3, 1.2, 1.0, 0.2, 0.5, 0.4]);
        JointTarget end = new([0.4, 1.25, 1.0, 0.25, 0.55, 0.4]);
        var plane = robot.Kinematics([end])[0].Planes[^1];
        Program reference = new("P", robot, [TestRobots.Toolpath(start, new CartesianTarget(plane, motion: motion))]);
        var expected = TestRobots.FlattenCode(reference);

        Plane basePlane = new(new(300, -200, 100), Vector3d.YAxis, -Vector3d.XAxis);
        Plane framePlane = new(new(-80, 250, 75), Vector3d.ZAxis, Vector3d.XAxis);
        robot.BasePlane = basePlane;
        plane.Orient(ref basePlane);
        plane.InverseOrient(ref framePlane);
        CartesianTarget target = new(plane, motion: motion, frame: new(framePlane));
        Program placed = new("P", robot, [TestRobots.Toolpath(start, target)]);

        Assert.Multiple(() =>
        {
            Assert.That(reference.Errors, Is.Empty);
            Assert.That(placed.Errors, Is.Empty);
            Assert.That(TestRobots.FlattenCode(placed), Is.EqualTo(expected));
        });
    }

    [Test]
    public void JakaCartesianJointMoveMatchesJointTarget()
    {
        var robot = TestRobots.SphericalRobot(Manufacturers.Jaka);
        JointTarget start = new([0.3, 1.2, 1.0, 0.2, 0.5, 0.4]);
        JointTarget end = new([0.4, 1.25, 1.0, 0.25, 0.55, 0.4]);
        var plane = robot.Kinematics([end])[0].Planes[^1];
        Program joint = new("P", robot, [TestRobots.Toolpath(start, end)]);
        Program cartesian = new("P", robot, [TestRobots.Toolpath(start, new CartesianTarget(plane, motion: Motions.Joint))]);

        Assert.Multiple(() =>
        {
            Assert.That(joint.Errors, Is.Empty);
            Assert.That(cartesian.Errors, Is.Empty);
            Assert.That(TestRobots.FlattenCode(cartesian), Is.EqualTo(TestRobots.FlattenCode(joint)));
        });
    }

    [TestCase(Manufacturers.Jaka)]
    [TestCase(Manufacturers.Igus)]
    public void BaseOnlyPostProcessorsRejectControllerFrames(Manufacturers manufacturer)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, 6);
        Frame frame = new(Plane.WorldXY, name: "ControllerFrame", useController: true);
        JointTarget target = new(new double[6], frame: frame);
        Program program = new("P", robot, [TestRobots.Toolpath(target)]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.Contains("Controller frames are not supported"));
    }

    [Test]
    public void FlyByTargetCommandsStayUnsynchronizedInPostProcessors()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.ABB, 6);
        JointTarget target = new(new double[6], zone: new(100), command: new SetDO(0, true));

        Program program = new("P", robot, [TestRobots.Toolpath(target)]);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Targets[0].ProgramTargets[0].Target.Zone.IsFlyBy, Is.True);
            Assert.That(program.Warnings, Has.Some.Contains("Commands on a fly-by target may run before or after the exact target position"));
            Assert.That(code, Does.Contain("SetDO DO1,1;"));
            Assert.That(code, Does.Not.Contain(@"SetDO \Sync"));
        });
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
        var mainCode = string.Join("\n", code[0][0]).UseLF();

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
    public void KukaJointSpeedTracksItsLeadingAxis()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.KUKA, 6);
        Speed speed = new(time: 1);
        JointTarget end = new([Math.PI / 2, Math.PI / 2, 0, 0, 0, 0], speed: speed);
        var targets = TestRobots.Toolpath(
            new JointTarget(new double[6]),
            new JointTarget([Math.PI / 2, 0, 0, 0, 0, 0], speed: speed),
            end, end,
            new JointTarget([Math.PI / 2, Math.PI, 0, 0, 0, 0], speed: speed));
        Program program = new("P", robot, [targets]);
        Assert.That(program.Errors, Is.Empty);

        var velocities = TestRobots.FlattenCode(program).Split('\n').Where(line => line.StartsWith("$VEL_AXIS[", StringComparison.Ordinal)).ToArray();
        string[] expected = ["$VEL_AXIS[1] = 50", "$VEL_AXIS[2] = 50"];
        Assert.That(velocities, Is.EqualTo(expected));
    }

    [Test]
    public void KukaJointSpeedIsRestoredAfterExternalMotion()
    {
        var robot = TestRobots.KukaWithCustomExternal();
        Speed speed = new(time: 1);
        var targets = TestRobots.Toolpath(
            new JointTarget(new double[6], external: [0]),
            new JointTarget([Math.PI / 2, 0, 0, 0, 0, 0], speed: speed, external: [0]),
            new JointTarget([Math.PI / 2, 0, 0, 0, 0, 0], external: [100]),
            new JointTarget([Math.PI, 0, 0, 0, 0, 0], speed: speed, external: [100]));
        Program program = new("P", robot, [targets]);
        Assert.That(program.Errors, Is.Empty);

        var code = TestRobots.FlattenCode(program).Split('\n');
        Assert.That(code.Count(line => line == "$VEL_AXIS[1] = 50"), Is.EqualTo(2));
        Assert.That(code.Count(line => line == "BAS(#VEL_PTP, 100)"), Is.EqualTo(2));
    }

    [TestCase(false)]
    [TestCase(true)]
    public void KukaDeclaresCustomExternalsForTheirGroup(bool firstTarget)
    {
        var robot = TestRobots.KukaTwoGroupWithCustomExternal();
        double[] joints = [0, 1, 1, 0, 0.5, 0];
        JointTarget target = new(joints);
        var external = TestRobots.Toolpath(
            new JointTarget(joints, external: [0], externalCustom: firstTarget ? ["0"] : null),
            new JointTarget(joints, external: [1], externalCustom: ["1"]));
        Program program = new("P", robot, [TestRobots.Toolpath(target, target), external]);
        Assert.That(program.Errors, Is.Empty);

        var code = program.Code!;
        Assert.That(string.Join('\n', code[0][1]), Does.Not.Contain("DECL GLOBAL E6"));
        Assert.That(string.Join('\n', code[1][1]), Does.Contain("DECL GLOBAL E6AXIS A").And.Contain("DECL GLOBAL E6POS P"));
        Assert.That(string.Join('\n', code[1][2]), Does.Contain("A.E1 = 1\nPTP A"));
    }

    [TestCase(-135, -2)]
    [TestCase(-90.000001, -2)]
    [TestCase(-89.999999, -1)]
    [TestCase(-45, -1)]
    [TestCase(45, 0)]
    [TestCase(89.999999, 0)]
    [TestCase(90.000001, 1)]
    [TestCase(135, 1)]
    public void AbbConfigurationUsesSignedQuarterTurns(double degrees, int quadrant)
    {
        var robot = TestRobots.AbbIrb120();
        double angle = degrees.ToRadians();
        JointTarget start = new([angle, 0.5, 0.2, angle, 0.6, angle]);
        var plane = robot.Kinematics([start])[0].Planes[^1];
        Program program = new("P", robot, [TestRobots.Toolpath(start, new CartesianTarget(plane, motion: Motions.Joint))]);
        Assert.That(program.Errors, Is.Empty);

        var motion = TestRobots.FlattenCode(program).Split('\n').Single(line => line.StartsWith("MoveJ ", StringComparison.Ordinal));
        Assert.That(motion, Does.Contain($"],[{quadrant},{quadrant},{quadrant},"));
    }

    [TestCase(false, "false")]
    [TestCase(true, "true")]
    public void StaubliWaitUsesInputArrayIndex(bool value, string expected)
    {
        var program = CreateProgram(Manufacturers.Staubli, 6, new WaitDI(0, value));
        Assert.That(program.Errors, Is.Empty);
        Assert.That(TestRobots.FlattenCode(program), Does.Contain($"wait(dis[0] == {expected})"));
    }

    [Test]
    public void JakaAnalogOutputDoesNotRequireDeclarations()
    {
        var program = CreateProgram(Manufacturers.Jaka, 6, new SetAO(0, 0.5));
        Assert.That(program.Errors, Is.Empty);
        Assert.That(TestRobots.FlattenCode(program), Does.Contain("set_analog_output(1,0,0.5,0)"));
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
    public void AbbTaskListMatchesMechanicalGroups()
    {
        var robot = TestRobots.AbbThreeGroup();
        var target = new JointTarget(new double[6]);
        var program = new Program(
            "P",
            robot,
            [
                TestRobots.Toolpath(target),
                TestRobots.Toolpath(target),
                TestRobots.Toolpath(target)
            ]);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(code, Does.Contain(
                @"TASK PERS tasks all_tasks{3} := [[""T_ROB1""], [""T_ROB2""], [""T_ROB3""]];"));
            Assert.That(code, Does.Not.Contain("all_tasks{2}"));
        });
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

    [TestCase(false)]
    [TestCase(true)]
    public void UrCustomFrameIsResolvedAtRuntime(bool useController)
    {
        var robot = TestRobots.UR10();
        JointTarget start = new([2.2208, -2.4093, 2.5006, 3.0503, 0.9208, -3.1416]);
        var frameName = useController ? "ControllerFrame" : "RuntimeFrame";
        Frame frame = new(Plane.WorldXY.WithOrigin(100, 0, 0), name: frameName, useController: useController);
        Plane localPlane = Plane.WorldZX.WithOrigin(600, 250, 600);
        CartesianTarget target = new(localPlane, motion: Motions.Linear, frame: frame);
        Program program = new("P", robot, [TestRobots.Toolpath(start, target)]);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(code, Does.Contain($"movel(pose_trans({frameName}, p[0.6, 0.25, 0.6"));
            Assert.That(code.Contains($"{frameName} = p[0.1, 0, 0, 0, 0, 0]", StringComparison.Ordinal), Is.EqualTo(!useController));
        });
    }

    [Test]
    public void UrpPreservesScriptWithXmlCharacters()
    {
        const string script = "def Program():\n  if 1 < 2:\n    textmsg(\"A & B > C ' ]]>\")\n  end\nend";
        CustomProgram program = new("P", TestRobots.UR10(), [0], [[[script]]]);
        var document = XDocument.Parse(UrProgramFile.CreateUrp(program));

        Assert.That(document.Descendants("cachedContents").Single().Value, Is.EqualTo(script + "\nProgram()\n"));
    }

    [Test]
    public void UrProcessMotionWithTimeFails()
    {
        var speed = new Speed(time: 2);
        var program = CreateProcessProgram(TestRobots.UR10(), speed);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Process motion does not support time-based speed on UR robots."));
    }

    [Test]
    public void UnsupportedPostProcessorRejectsProcessMotion()
    {
        var program = CreateProcessProgram(TestRobots.AbbIrb120());

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Process motion is not supported by RapidPostProcessor."));
    }

    static Program CreateProcessProgram(RobotSystem robot, Speed? speed = null)
    {
        var start = new JointTarget(new double[6]);
        var endPlane = robot.Kinematics([start])[0].Planes[^1];
        var processTarget = new CartesianTarget(endPlane, motion: Motions.Process, speed: speed);

        return new("P", robot, [TestRobots.Toolpath(start, processTarget)]);
    }

    static Program CreateProgram(Manufacturers manufacturer, int jointCount, Command? command = null)
    {
        var robot = TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var target = new JointTarget(new double[jointCount], command: command);
        return new("P", robot, [TestRobots.Toolpath(target)]);
    }

    class SavingPostProcessor : IPostProcessor
    {
        public IProgram? Program { get; private set; }
        public string? Folder { get; private set; }

        public List<List<List<string>>> GetCode(RobotSystem system, Program program) => [[[]]];

        public void Save(IProgram program, string folder)
        {
            Program = program;
            Folder = folder;
        }
    }

    sealed class SuppressingFormatter : CommandFormatter
    {
        protected override string? FormatCommand(Command command, RobotSystem system, Target target) =>
            command is Stop ? "" : null;
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
