using System.Globalization;
using NUnit.Framework;
using Rhino.Geometry;
using Robots.Commands;
using Regex = System.Text.RegularExpressions.Regex;

namespace Robots.Tests;

class FrankyPostProcessorTests
{
    static readonly double[] _start = [0, -0.7, 0.1, -1.8, 0.2, 1.5, 0.7];
    static readonly double[] _first = [0.05, -0.68, 0.11, -1.75, 0.18, 1.48, 0.72];
    static readonly double[] _second = [0.1, -0.65, 0.12, -1.7, 0.16, 1.46, 0.74];
    static readonly double[] _third = [0.15, -0.62, 0.13, -1.65, 0.14, 1.44, 0.76];

    [Test]
    public void FrankxRemainsDefault()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7);

        Assert.That(robot.PostProcessor, Is.TypeOf<FrankxPostProcessor>());
    }

    [Test]
    public void XmlRejectsUnknownPostProcessor()
    {
        var exception = Assert.Throws<ArgumentException>(() =>
            TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7, "UnknownPostProcessor"));

        Assert.That(exception!.Message, Is.EqualTo("Post processor 'UnknownPostProcessor' was not found in the Robots assembly."));
    }

    [Test]
    public void PostProcessorValidatesRobotSystemCompatibility()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7, nameof(RapidPostProcessor));
        var exception = Assert.Throws<ArgumentException>(() =>
            new Program("P", robot, [TestRobots.Toolpath(new JointTarget(_start))]));

        Assert.Multiple(() =>
        {
            Assert.That(exception!.Message, Does.Contain("The RAPID post processor requires an ABB robot system."));
            Assert.That(exception.ParamName, Is.EqualTo("system"));
        });
    }

    [Test]
    public void XmlRejectsTypeThatIsNotPostProcessor()
    {
        var exception = Assert.Throws<ArgumentException>(() =>
            TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7, nameof(SystemFranka)));

        Assert.That(exception!.Message, Is.EqualTo("Post processor 'SystemFranka' must be a concrete IPostProcessor implementation."));
    }

    [Test]
    public void ExplicitPostProcessorOverridesXml()
    {
        StubPostProcessor postProcessor = new();
        var robot = TestRobots.PostProcessorRobot(
            Manufacturers.FrankaEmika,
            7,
            "UnknownPostProcessor",
            postProcessor);

        Assert.That(robot.PostProcessor, Is.SameAs(postProcessor));
    }

    [Test]
    public void GeneratesFrankyJointMotion()
    {
        var program = new Program("P", Robot(), [TestRobots.Toolpath(new JointTarget(_start))]);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(code, Does.Contain("from franky import"));
            Assert.That(code, Does.Not.Contain("from frankx import"));
            Assert.That(code, Does.Contain("if not robot.recover_from_errors():"));
            Assert.That(code, Does.Contain("robot.set_ee([1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1])"));
            Assert.That(code, Does.Not.Contain("velocity_limit.get()"));
            Assert.That(code, Does.Contain("_robots_tool_000 = Affine("));
            Assert.That(code, Does.Contain("motion = JointMotion([0, -0.7, 0.1, -1.8, 0.2, 1.5, 0.7]"));
        });
    }

    [Test]
    public void ReadsRuntimeVelocityLimitsAfterCommandsForEachMotion()
    {
        const string initCode = "robot.translation_velocity_limit.set(0.5)";
        const string targetCode = "robot.translation_velocity_limit.set(0.25)";
        const string getter = "robot.translation_velocity_limit.get()";
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var start = TargetPlane(robot, _start);
        var first = start.WithOrigin(start.Origin + start.XAxis * 10);
        var second = start.WithOrigin(start.Origin + start.XAxis * 20);
        var init = new Group([new Commands.Custom(command: initCode)]);
        var beforeSecond = new Commands.Custom(command: targetCode, runBefore: true);
        Target[] targets =
        [
            new JointTarget(_start),
            new CartesianTarget(first, motion: Motions.Linear, external: [_start[2]]),
            new CartesianTarget(second, motion: Motions.Linear, command: beforeSecond, external: [_start[2]])
        ];
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(targets)],
            initCommands: init,
            stepSize: 1000);
        var code = TestRobots.FlattenCode(program);

        int initIndex = code.IndexOf(initCode, StringComparison.Ordinal);
        int firstReadIndex = code.IndexOf(getter, initIndex + initCode.Length, StringComparison.Ordinal);
        int targetIndex = code.IndexOf(targetCode, firstReadIndex + getter.Length, StringComparison.Ordinal);
        int secondReadIndex = code.IndexOf(getter, targetIndex + targetCode.Length, StringComparison.Ordinal);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(initIndex, Is.GreaterThanOrEqualTo(0));
            Assert.That(firstReadIndex, Is.GreaterThan(initIndex));
            Assert.That(targetIndex, Is.GreaterThan(firstReadIndex));
            Assert.That(secondReadIndex, Is.GreaterThan(targetIndex));
            Assert.That(Count(code, getter), Is.EqualTo(2));
            Assert.That(code, Does.Not.Contain("_robots_translation_velocity_limit"));
        });
    }

    [Test]
    public void CartesianJointTargetGeneratesJointMotion()
    {
        var robot = Robot();
        var target = new CartesianTarget(
            TargetPlane(robot, _first),
            motion: Motions.Joint,
            external: [_first[2]]);
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(_start), target)],
            stepSize: 1000);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(Count(code, "motion = JointMotion("), Is.EqualTo(2));
            Assert.That(code, Does.Not.Contain("motion = CartesianMotion("));
        });
    }

    [Test]
    public void CartesianWithoutExternalLeavesElbowFree()
    {
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var target = new CartesianTarget(
            TargetPlane(robot, _first),
            motion: Motions.Linear);
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(_start), target)],
            stepSize: 1000);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(robot.PostProcessor, Is.TypeOf<FrankyPostProcessor>());
            Assert.That(((SingleGroupSystem)robot).Robot.Solver, Is.TypeOf<FixedRedundancyKinematics>());
            Assert.That(code, Does.Contain("motion = CartesianMotion(Affine("));
            Assert.That(code, Does.Not.Contain("ElbowState("));
        });
    }

    [Test]
    public void ExplicitElbowAfterFreeCartesianRequiresJointTarget()
    {
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var free = new CartesianTarget(TargetPlane(robot, _first), motion: Motions.Linear);
        var explicitElbow = new CartesianTarget(
            TargetPlane(robot, _second),
            motion: Motions.Linear,
            external: [_second[2]]);
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(_start), free, explicitElbow)],
            stepSize: 1000);

        Assert.That(program.Code, Is.Null);
        Assert.That(
            program.Errors,
            Has.One.EqualTo("Target 2: A joint target is required before restoring an explicit Franka elbow after free-elbow Cartesian motion."));
    }

    [Test]
    public void SupportedPandaInitialCartesianUsesAnalyticalSolver()
    {
        var system = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var robot = ((SingleGroupSystem)system).Robot;
        double[] joints = [0.25, -0.65, robot.Joints[2].Range.Mid, -1.35, 0.8, 1.1, -0.55];
        var target = new CartesianTarget(
            TargetPlane(system, joints),
            motion: Motions.Linear);
        var program = new Program("P", system, [TestRobots.Toolpath(target)], stepSize: 1000);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Warnings, Has.Some.Contains("First target changed to a joint target."));
            Assert.That(code, Does.Contain("motion = JointMotion("));
        });
    }

    [Test]
    public void ProcessGeneratesContinuousWaypoints()
    {
        var robot = Robot();
        var program = ProcessProgram(robot, [_first, _second, _third]);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(Count(code, "CartesianWaypoint("), Is.EqualTo(3));
            Assert.That(Count(code, "CartesianState("), Is.EqualTo(2));
            Assert.That(Count(code, "RobotVelocity(Twist("), Is.EqualTo(2));
            Assert.That(Count(code, "CartesianWaypoint(RobotPose("), Is.EqualTo(1));
            Assert.That(code, Does.Contain("_robots_velocity_scale_001 = min("));
            Assert.That(code, Does.Contain("min(_robots_dynamics_001.velocity, _robots_dynamics_002.velocity)"));
            Assert.That(code, Does.Contain(" * _robots_velocity_scale_001"));
            Assert.That(code, Does.Contain("], ee_frame=_robots_tool_000)"));
        });
    }

    [Test]
    public void ProcessWithoutExternalLeavesElbowFree()
    {
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        double[] firstJoints = [.. _first];
        double[] secondJoints = [.. _second];
        firstJoints[2] = _start[2];
        secondJoints[2] = _start[2];
        Target[] targets =
        [
            new JointTarget(_start),
            new CartesianTarget(TargetPlane(robot, firstJoints), motion: Motions.Process),
            new CartesianTarget(TargetPlane(robot, secondJoints), motion: Motions.Process)
        ];
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(targets)],
            stepSize: 1000);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(code, Does.Contain("RobotVelocity(Twist("));
            Assert.That(code, Does.Not.Contain("ElbowState("));
            Assert.That(code, Does.Not.Contain("elbow_velocity="));
        });
    }

    [Test]
    public void ProcessVelocityRespectsAdjacentDynamics()
    {
        var slow = new Speed(1, rotationSpeed: 0.01);
        var fast = new Speed(1000, rotationSpeed: 2.5);
        var robot = Robot();
        var program = ProcessProgram(robot, [_first, _second], speeds: [slow, fast]);
        var code = TestRobots.FlattenCode(program);
        var rawVelocity = FirstRawVelocity(code);
        double factor = Math.Min(slow.TranslationSpeed / 1700, slow.RotationSpeed / 2.5);
        var first = TargetPlane(robot, _first);
        var velocity = ScaleVelocity(rawVelocity, first, Tool.Default, factor);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(code, Does.Contain(" * robot.translation_velocity_limit.get() /"));
            Assert.That(code, Does.Contain(" * robot.rotation_velocity_limit.get() /"));
            Assert.That(code, Does.Contain(" * robot.elbow_velocity_limit.get() /"));
            Assert.That(velocity.Linear.Length, Is.LessThanOrEqualTo(factor * 1.7 + 1e-9));
            Assert.That(velocity.Angular.Length, Is.LessThanOrEqualTo(factor * 2.5 + 1e-9));
            Assert.That(Math.Abs(velocity.Elbow), Is.LessThanOrEqualTo(factor * 2.175 + 1e-9));
        });
    }

    [Test]
    public void CartesianSpeedUsesOnlyActiveChannels()
    {
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var start = TargetPlane(robot, _start);
        var translated = start.WithOrigin(start.Origin + start.XAxis * 10);
        var rotated = start;
        _ = rotated.Rotate(0.02, start.ZAxis, start.Origin);
        var mixed = rotated.WithOrigin(rotated.Origin + rotated.XAxis * 10);

        string translationWithSlowRotation = CartesianSpeed(
            LinearPlaneProgram(robot, translated, new(170, rotationSpeed: 0.001)));
        string translationWithFastRotation = CartesianSpeed(
            LinearPlaneProgram(robot, translated, new(170, rotationSpeed: 2.5)));
        string rotationWithSlowTranslation = CartesianSpeed(
            LinearPlaneProgram(robot, rotated, new(1, rotationSpeed: 0.25)));
        string rotationWithFastTranslation = CartesianSpeed(
            LinearPlaneProgram(robot, rotated, new(1700, rotationSpeed: 0.25)));
        string mixedSpeed = CartesianSpeed(
            LinearPlaneProgram(robot, mixed, new(170, rotationSpeed: 0.5)));

        Assert.Multiple(() =>
        {
            Assert.That(
                translationWithSlowRotation,
                Is.EqualTo("min(1, 0.17 / robot.translation_velocity_limit.get())"));
            Assert.That(translationWithFastRotation, Is.EqualTo(translationWithSlowRotation));
            Assert.That(
                rotationWithSlowTranslation,
                Is.EqualTo("min(1, 0.25 / robot.rotation_velocity_limit.get())"));
            Assert.That(rotationWithFastTranslation, Is.EqualTo(rotationWithSlowTranslation));
            Assert.That(
                mixedSpeed,
                Is.EqualTo("min(1, 0.17 / robot.translation_velocity_limit.get(), 0.5 / robot.rotation_velocity_limit.get())"));
        });
    }

    [Test]
    public void PreservesPositiveDynamicsFactorsBelowOneMillionth()
    {
        var jointSpeed = new Speed(0.0001, axisAccel: 4 * Math.PI * 1e-8);
        var jointProgram = new Program(
            "J",
            Robot(),
            [TestRobots.Toolpath(new JointTarget(_start, speed: jointSpeed))]);
        var jointCode = TestRobots.FlattenCode(jointProgram);

        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var start = TargetPlane(robot, _start);
        var translated = start.WithOrigin(start.Origin + start.XAxis * 10);
        var cartesianProgram = LinearPlaneProgram(
            robot,
            translated,
            new Speed(0.0001, axisAccel: 4 * Math.PI * 1e-8));

        Assert.Multiple(() =>
        {
            Assert.That(jointProgram.Errors, Is.Empty);
            Assert.That(
                jointCode,
                Does.Contain("relative_dynamics_factor=RelativeDynamicsFactor(1E-07, 1E-08, 1E-08)"));
            Assert.That(cartesianProgram.Errors, Is.Empty);
            Assert.That(
                CartesianSpeed(cartesianProgram),
                Is.EqualTo("min(1, 1E-07 / robot.translation_velocity_limit.get())"));
        });
    }

    [Test]
    public void ProcessAngularVelocityIgnoresInactiveTranslationSpeed()
    {
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        var first = TargetPlane(robot, _start);
        var second = first;
        _ = second.Rotate(0.02, first.ZAxis, first.Origin);
        var slowTranslation = ProcessPlaneProgram(robot, [first, second], speed: new(1, rotationSpeed: 0.25));
        var fastTranslation = ProcessPlaneProgram(robot, [first, second], speed: new(1700, rotationSpeed: 0.25));
        var slowCode = TestRobots.FlattenCode(slowTranslation);
        var fastCode = TestRobots.FlattenCode(fastTranslation);
        var slowVelocity = FirstRawVelocity(slowCode);

        Assert.Multiple(() =>
        {
            Assert.That(slowTranslation.Errors, Is.Empty);
            Assert.That(fastTranslation.Errors, Is.Empty);
            Assert.That(slowVelocity.Angular.Length, Is.GreaterThan(1e-6));
            Assert.That(slowCode, Is.EqualTo(fastCode));
            Assert.That(slowCode, Does.Not.Contain(" / robot.translation_velocity_limit.get()"));
        });
    }

    [Test]
    public void ProcessKeepsSmallAngularVelocity()
    {
        var robot = Robot();
        var first = TargetPlane(robot, _first);
        var second = first;
        _ = second.Rotate(0.001, first.ZAxis, first.Origin);
        var program = ProcessPlaneProgram(robot, [first, second]);
        var velocity = FirstRawVelocity(TestRobots.FlattenCode(program));

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(velocity.Angular.Length, Is.GreaterThan(1e-6));
        });
    }

    [Test]
    public void ProcessVelocityAccountsForToolOffset()
    {
        var robot = Robot();
        var tool = new Tool(Plane.WorldXY.WithOrigin(1000, 0, 0), "OffsetTool");
        var first = TargetPlane(robot, _first, tool);
        var second = first;
        _ = second.Rotate(0.02, first.ZAxis, first.Origin);
        var program = ProcessPlaneProgram(robot, [first, second], tool);
        var code = TestRobots.FlattenCode(program);
        var rawVelocity = FirstRawVelocity(code);
        double dynamics = 100.0 / 1700;
        var velocity = ScaleVelocity(rawVelocity, first, tool, dynamics);
        var eeLinear = EndEffectorLinear(first, tool, velocity);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(velocity.Angular.Length, Is.GreaterThan(1e-6));
            Assert.That(eeLinear.Length, Is.GreaterThan(dynamics * 1.5));
            Assert.That(eeLinear.Length, Is.LessThanOrEqualTo(dynamics * 1.7));
        });
    }

    [Test]
    public void ToolNamesCannotCollideWithPython()
    {
        var robot = Robot();
        var tool = new Tool(Plane.WorldXY.WithOrigin(100, 0, 0), "class");
        var target = new CartesianTarget(
            TargetPlane(robot, _first, tool),
            motion: Motions.Linear,
            tool: tool,
            external: [_first[2]]);
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(_start, tool), target)],
            stepSize: 1000);
        var code = TestRobots.FlattenCode(program);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(code, Does.Contain("_robots_tool_000 = Affine("));
            Assert.That(code, Does.Contain("ee_frame=_robots_tool_000"));
            Assert.That(code, Does.Not.Contain("class ="));
        });
    }

    [Test]
    public void RejectsZones()
    {
        var robot = Robot();
        var distance = ProcessProgram(robot, [_first], zone: new Zone(1));
        var rotation = ProcessProgram(robot, [_first], zone: new Zone(0, rotation: 0.1));

        Assert.Multiple(() =>
        {
            Assert.That(distance.Code, Is.Null);
            Assert.That(distance.Errors, Has.One.EqualTo("Target 1: Zones are not supported by the Franky postprocessor."));
            Assert.That(rotation.Code, Is.Null);
            Assert.That(rotation.Errors, Has.One.EqualTo("Target 1: Zones are not supported by the Franky postprocessor."));
        });
    }

    [Test]
    public void RejectsTimeBasedSpeed()
    {
        var program = ProcessProgram(Robot(), [_first], speed: new Speed(time: 1));

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Time-based speeds are not supported by the Franky postprocessor."));
    }

    [Test]
    public void RejectsUnsupportedToolsAndFrames()
    {
        var payload = new Program(
            "P",
            Robot(),
            [TestRobots.Toolpath(new JointTarget(_start, tool: new(Plane.WorldXY, weight: 1)))]);
        var controllerTool = new Program(
            "P",
            Robot(),
            [TestRobots.Toolpath(new JointTarget(_start, tool: new(Plane.WorldXY, useController: true)))]);
        var controllerFrame = new Program(
            "P",
            Robot(),
            [TestRobots.Toolpath(new JointTarget(_start, frame: new(Plane.WorldXY, useController: true)))]);

        Assert.Multiple(() =>
        {
            Assert.That(payload.Errors, Has.One.EqualTo("Target 0: Tool payloads are not supported by the Franky postprocessor."));
            Assert.That(controllerTool.Errors, Has.One.EqualTo("Target 0: Controller tools are not supported by the Franky postprocessor."));
            Assert.That(controllerFrame.Errors, Has.One.EqualTo("Target 0: Controller frames are not supported by the Franky postprocessor."));
        });
    }

    [Test]
    public void FrankxRejectsProcess()
    {
        var robot = TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7);
        var program = ProcessProgram(robot, [_first]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Process motion is not supported by Frankx."));
    }

    [Test]
    public void RejectsCommandInsideProcessSequence()
    {
        var robot = Robot();
        var command = new Message("Interrupt", runBefore: true);
        var program = ProcessProgram(robot, [_first, _second, _third], command, 1);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 2: Commands cannot interrupt a continuous Process motion."));
    }

    [Test]
    public void RejectsCommandAfterProcessWaypoint()
    {
        var robot = Robot();
        var command = new Message("Interrupt");
        var program = ProcessProgram(robot, [_first, _second, _third], command, 1);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 2: Commands cannot interrupt a continuous Process motion."));
    }

    [Test]
    public void RejectsToolChangeInsideProcessSequence()
    {
        var robot = Robot();
        var firstTool = new Tool(Plane.WorldXY.WithOrigin(0, 0, 50), "FirstTool");
        var secondTool = new Tool(Plane.WorldXY.WithOrigin(0, 0, 100), "SecondTool");
        Target[] targets =
        [
            new JointTarget(_start, firstTool),
            new CartesianTarget(
                TargetPlane(robot, _first, firstTool),
                motion: Motions.Process,
                tool: firstTool,
                external: [_first[2]]),
            new CartesianTarget(
                TargetPlane(robot, _second, secondTool),
                motion: Motions.Process,
                tool: secondTool,
                external: [_second[2]])
        ];
        var program = new Program("P", robot, [TestRobots.Toolpath(targets)], stepSize: 1000);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Tool changes cannot interrupt a continuous Process motion."));
    }

    [Test]
    public void RejectsRepeatedProcessTargetWithoutThrowing()
    {
        var robot = Robot();
        var program = ProcessProgram(robot, [_first, _first]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 2: Consecutive Process targets must not be identical."));
    }

    [Test]
    public void RejectsCartesianMotionAtFlipBoundary()
    {
        var robot = Robot();
        var joints = (double[])_start.Clone();
        joints[3] = -0.467002423653011;
        var plane = TargetPlane(robot, joints);
        var target = new CartesianTarget(plane, motion: Motions.Linear, external: [joints[2]]);
        var program = new Program("P", robot, [TestRobots.Toolpath(new JointTarget(joints), target)], stepSize: 1000);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Cartesian motion at the Franka elbow flip boundary is not supported by Franky."));
    }

    [Test]
    public void RejectsCartesianElbowFlipChange()
    {
        var robot = TestRobots.FrankaPanda(nameof(FrankyPostProcessor));
        double[] previous = [0, -0.7, 0.1, -0.6, 0.2, 1.5, 0.7];
        double[] joints = [0, -0.7, 0.1, -0.3, 0.2, 1.5, 0.7];
        var target = new CartesianTarget(
            TargetPlane(robot, joints),
            motion: Motions.Linear,
            external: [joints[2]]);
        var program = new Program(
            "P",
            robot,
            [TestRobots.Toolpath(new JointTarget(previous), new JointTarget(joints))]);
        Assert.That(program.Errors, Is.Empty);

        // Exercise controller policy independently of Cartesian path validation.
        program.Targets[1].ProgramTargets[0].Target = target;
        _ = robot.PostProcessor.GetCode(robot, program);
        Assert.That(program.Errors, Has.One.EqualTo("Target 1: Cartesian motion cannot change the Franka elbow flip direction with Franky."));
    }

    [Test]
    public void NumericalFallbackRequiresInitialJointTarget()
    {
        var robot = Robot();
        var target = new CartesianTarget(
            TargetPlane(robot, _first),
            motion: Motions.Linear,
            external: [_first[2]]);
        var program = new Program("P", robot, [TestRobots.Toolpath(target)]);

        Assert.That(program.Code, Is.Null);
        Assert.That(program.Errors, Has.One.EqualTo("Target 0: First target should be a joint target because this robot needs a known starting joint state."));
    }

    static RobotSystem Robot() =>
        TestRobots.PostProcessorRobot(Manufacturers.FrankaEmika, 7, nameof(FrankyPostProcessor));

    static Program ProcessProgram(
        RobotSystem robot,
        IReadOnlyList<double[]> waypoints,
        Command? command = null,
        int commandIndex = -1,
        Speed? speed = null,
        Zone? zone = null,
        IReadOnlyList<Speed>? speeds = null)
    {
        var targets = new List<Target> { new JointTarget(_start) };

        for (int i = 0; i < waypoints.Count; i++)
        {
            var joints = waypoints[i];
            targets.Add(new CartesianTarget(
                TargetPlane(robot, joints),
                motion: Motions.Process,
                speed: speeds?[i] ?? speed,
                zone: zone,
                command: i == commandIndex ? command : null,
                external: [joints[2]]));
        }

        return new("P", robot, [TestRobots.Toolpath([.. targets])], stepSize: 1000);
    }

    static Program ProcessPlaneProgram(RobotSystem robot, IReadOnlyList<Plane> planes, Tool? tool = null, Speed? speed = null)
    {
        var targets = new List<Target> { new JointTarget(_start, tool) };

        foreach (var plane in planes)
        {
            targets.Add(new CartesianTarget(
                plane,
                motion: Motions.Process,
                tool: tool,
                speed: speed,
                external: [_first[2]]));
        }

        return new("P", robot, [TestRobots.Toolpath([.. targets])], stepSize: 1000);
    }

    static Program LinearPlaneProgram(RobotSystem robot, Plane plane, Speed speed)
    {
        Target[] targets =
        [
            new JointTarget(_start),
            new CartesianTarget(plane, motion: Motions.Linear, speed: speed, external: [_start[2]])
        ];
        return new("P", robot, [TestRobots.Toolpath(targets)], stepSize: 1000);
    }

    static Plane TargetPlane(RobotSystem robot, double[] joints, Tool? tool = null) =>
        robot.Kinematics([new JointTarget(joints, tool)])[0].Planes[^1];

    static ParsedVelocity FirstRawVelocity(string code)
    {
        var match = Regex.Match(
            code,
            @"RobotVelocity\(Twist\(\[([^\]]+)\], \[([^\]]+)\]\), elbow_velocity=([^\)]+)\)");

        Assert.That(match.Success, Is.True);
        return new(
            Vector(match.Groups[1].Value),
            Vector(match.Groups[2].Value),
            NumberExpression(match.Groups[3].Value));
    }

    static string CartesianSpeed(Program program)
    {
        Assert.That(program.Errors, Is.Empty);
        var code = TestRobots.FlattenCode(program);
        const string marker = "relative_dynamics_factor=RelativeDynamicsFactor(";
        int motion = code.IndexOf("motion = CartesianMotion(", StringComparison.Ordinal);

        Assert.That(motion, Is.GreaterThanOrEqualTo(0));
        int start = code.IndexOf(marker, motion, StringComparison.Ordinal) + marker.Length;

        Assert.That(start, Is.GreaterThanOrEqualTo(marker.Length));
        int depth = 0;

        for (int i = start; i < code.Length; i++)
        {
            if (code[i] == '(')
            {
                depth++;
            }
            else if (code[i] == ')')
            {
                depth--;
            }
            else if (code[i] == ',' && depth == 0)
            {
                return code[start..i];
            }
        }

        Assert.Fail("Cartesian dynamics speed was not found.");
        return "";
    }

    static Vector3d EndEffectorLinear(Plane tcp, Tool tool, ParsedVelocity velocity)
    {
        Plane defaultEe = new(Point3d.Origin, -Vector3d.XAxis, Vector3d.YAxis);
        var eeFrame = tool.Tcp;
        eeFrame.Orient(ref defaultEe);
        var transform = tcp.ToTransform() * eeFrame.ToInverseTransform();
        var eePlane = transform.ToPlane();
        var offset = (eePlane.Origin - tcp.Origin) * 0.001;
        return velocity.Linear + Vector3d.CrossProduct(velocity.Angular, offset);
    }

    static Vector3d Vector(string values)
    {
        var numbers = values.Split(',').Select(NumberExpression).ToArray();
        Assert.That(numbers, Has.Length.EqualTo(3));
        return new(numbers[0], numbers[1], numbers[2]);
    }

    static double Number(string value) =>
        double.Parse(value, CultureInfo.InvariantCulture);

    static double NumberExpression(string value) =>
        Number(value.Split('*', 2, StringSplitOptions.TrimEntries)[0]);

    static ParsedVelocity ScaleVelocity(
        ParsedVelocity velocity,
        Plane tcp,
        Tool tool,
        double dynamics)
    {
        const double margin = 0.99;
        var eeLinear = EndEffectorLinear(tcp, tool, velocity);
        double scale = 1;

        scale = Limit(scale, eeLinear.Length, margin * dynamics * 1.7);
        scale = Limit(scale, velocity.Angular.Length, margin * dynamics * 2.5);
        scale = Limit(scale, Math.Abs(velocity.Elbow), margin * dynamics * 2.175);

        return new(velocity.Linear * scale, velocity.Angular * scale, velocity.Elbow * scale);

        static double Limit(double scale, double value, double limit)
        {
            return value > limit ? Math.Min(scale, limit / value) : scale;
        }
    }

    static int Count(string value, string part) =>
        value.Split(part, StringSplitOptions.None).Length - 1;

    readonly record struct ParsedVelocity(Vector3d Linear, Vector3d Angular, double Elbow);

    class StubPostProcessor : IPostProcessor
    {
        public List<List<List<string>>> GetCode(RobotSystem system, Program program) => [[[]]];
    }
}
