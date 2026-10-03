using NUnit.Framework;
using Rhino.Geometry;
using Robots.Commands;

namespace Robots.Tests;

public class MotionPlannerTests
{
    [Test]
    public void LinearVerificationStartsFromThePreviousPose()
    {
        var robot = TestRobots.AbbPowa1920();
        JointTarget home = new([
            -0.3783987291987979, 1.9774851507868083, -0.9254284377328252,
            1.2046489047839533, 0.21653292128654789, -1.1041875577085594]);

        JointTarget endJoints = new([
            0.8538004950870761, 1.0593833907318224, 0.0162207547185107,
            1.8275292521470827, -0.06480278659835581, -2.198246811329502]);

        CartesianTarget end = new(robot.Kinematics([endJoints])[0].Planes[^1], motion: Motions.Linear);
        Program program = new("FirstSample", robot, [TestRobots.Toolpath(home, end)]);
        Assert.That(program.Errors, Is.Empty);
        var first = program.MotionSegments[0];
        var expected = robot.Kinematics(first.Lerp(robot, first.End.TotalTime), [home.Joints])[0];

        Assert.That(first.End.Joints, Is.EqualTo(expected.Joints).Within(1e-6));
        program.Animate(first.End.TotalTime - 0.001, false);
        var before = program.CurrentSimulationPose.Kinematics[0].Joints;
        program.Animate(first.End.TotalTime, false);
        Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints.Zip(before, (a, b) => Math.Abs(a - b)).Max(),
            Is.LessThan(0.01));
    }

    [Test]
    public void FlyByCheckingAndSimulationFollowTheSameCurve()
    {
        var robot = TestRobots.AbbIrb120();
        double[][] joints =
        [
            [
                -0.13559230185839927, 1.959811306865798, 0.8580524961734436,
                -1.0809059020042913, 0.14875505587493765, -0.2548102983528796
            ],
            [
                1.205748764009098, 1.5433115001038238, -0.0991007938511207,
                -0.7116290280649573, 0.037297215283521035, 0.7661208249470782
            ],
            [
                0.34959468033611535, 0.6514845347737357, 0.7792090049848003,
                -0.7984384322997362, 0.3833138292111986, 1.490660287761437
            ]
        ];

        var targets = joints.Select((values, i) => i == 0
            ? (Target)new JointTarget(values)
            : new CartesianTarget(robot.Kinematics([new JointTarget(values)])[0].Planes[^1],
                motion: Motions.Linear, zone: new(i == 1 ? 60 : 0))).ToArray();

        Program program = new("BlendCheck", robot, [new SimpleToolpath(targets)]);
        Assert.That(program.Errors, Is.Empty);
        var blend = program.MotionSegments.Single(segment => segment.Corner is not null);
        Assert.That(blend.CheckedSamples, Is.Not.Null);
        var samples = blend.CheckedSamples!;
        Assert.That(samples, Has.Length.GreaterThan(2));

        const int divisions = 20;
        var expected = new double[divisions + 1][];

        for (int i = 0; i <= divisions; i++)
        {
            double time = blend.Start.TotalTime + (blend.End.TotalTime - blend.Start.TotalTime) * i / divisions;
            program.Animate(time, false);
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Errors, Is.Empty);
            expected[i] = program.CurrentSimulationPose.Kinematics[0].Joints;
            var target = (CartesianTarget)blend.Lerp(robot, time)[0];
            Assert.That(program.CurrentSimulationPose.GetLastPlane(0).Origin.DistanceTo(target.Plane.Origin),
                Is.LessThan(1e-6));
        }

        for (int i = divisions; i >= 0; i--)
        {
            program.Animate(blend.Start.TotalTime + (blend.End.TotalTime - blend.Start.TotalTime) * i / divisions, false);
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints, Is.EqualTo(expected[i]).Within(1e-6));
        }

        double errorTime = samples[samples.Length / 2].TotalTime;
        var errorTarget = (CartesianTarget)blend.Lerp(robot, errorTime)[0];
        robot.GetRobot(0).Solver = new ErrorRegionKinematics(robot.GetRobot(0), errorTarget.Plane.Origin);
        Program invalid = new("BlendError", robot, [new SimpleToolpath(targets)]);
        Assert.That(invalid.Errors, Has.Some.Contains("Test blend failure"));
        Assert.That(invalid.Code, Is.Null);
        Assert.That(invalid.Duration, Is.EqualTo(errorTime).Within(1e-8));
        invalid.Animate(1);
        Assert.That(invalid.CurrentSimulationPose.Kinematics[0].Errors, Has.Some.Contains("Test blend failure"));
    }

    [Test]
    public void FixedTcpStillChecksMovingRobotBase()
    {
        const string positioner = """
            <Positioner model="Turntable" manufacturer="ABB" payload="1000" movesRobot="true">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints><Revolute number="7" a="0" d="0" minrange="-360" maxrange="360" maxspeed="90"/></Joints>
            </Positioner>
            """;

        string xml = TestRobots.AbbIrb120Xml.Replace("<Base x=\"0.000\"", "<Base x=\"300.000\"")
            .Replace("<RobotArm ", positioner + "<RobotArm ");

        var robot = FileIO.ParseRobotSystem(xml, Plane.WorldXY);
        JointTarget home = new([
            0.8921338360465842, 2.769254328206242, 0.9633252628784259,
            0.7707737526528244, -2.0194100303176903, 1.9694550617331068], external: [-Math.PI / 2]);

        CartesianTarget end = new(Plane.WorldYZ.WithOrigin(-300, 0, 600),
            motion: Motions.Linear, external: [Math.PI / 2]);

        var start = robot.Kinematics([home])[0];
        Assert.That(start.Planes[^1].Origin.DistanceTo(end.Plane.Origin), Is.LessThan(1e-6));
        Assert.That(start.Errors, Is.Empty);
        Assert.That(robot.Kinematics([end], [start.Joints])[0].Errors, Is.Empty);
        Program program = new("MovingBase", robot, [TestRobots.Toolpath(home, end)]);

        Assert.That(program.Errors, Is.Not.Empty);
        Assert.That(program.Code, Is.Null);
        Assert.That(program.MotionSegments[^1].End.Joints[^1], Is.LessThan(Math.PI / 2));
    }

    [TestCase(70, false)]
    [TestCase(119, false)]
    [TestCase(70, true)]
    public void LinearSimulationFollowsCheckedBranchRegardlessOfPlaybackOrder(double wristDegrees, bool flyby)
    {
        var system = TestRobots.AbbPowa1920();
        Tool tool = new(new(new(330.0908508300781, 0.17273835503788149, 121.14247027800656),
            Vector3d.YAxis, Vector3d.ZAxis));

        double wrist = wristDegrees * Math.PI / 180;
        JointTarget home = new([0, 2 * Math.PI / 3, -28 * Math.PI / 180, wrist, -8 * Math.PI / 180, -wrist],
            tool: tool, speed: new(10));

        Plane plane = new(new(1761.3116721944148, 1350, 916.9466015140285),
            new Vector3d(-0.6181225392792719, 0.485642931178632, -0.6181225362589292),
            new(0.34340141071070707, 0.8741572761215379, 0.34340140903273814));

        CartesianTarget end = new(plane, motion: Motions.Linear, tool: tool,
            speed: home.Speed, zone: new(flyby ? 1 : 0));

        Target[] targets = flyby
            ? [home, end, new CartesianTarget(plane.WithOrigin(1701.9147, 1350, 857.5496), end)]
            : [home, end];

        Program program = new("Continuity", system, [new SimpleToolpath(targets)]);
        Assert.That(program.Errors, Is.Empty);
        Assert.That(program.Warnings, Is.Empty);

        const int divisions = 100;
        var expected = new double[divisions + 1][];
        expected[0] = home.Joints;
        var startPlane = system.Kinematics([home])[0].Planes[^1];

        for (int i = 1; i <= divisions; i++)
        {
            var pose = system.CartesianLerp(startPlane, plane, i, 0, divisions);
            var solution = system.Kinematics([new CartesianTarget(pose, end)], [expected[i - 1]])[0];
            Assert.That(solution.Errors, Is.Empty);
            expected[i] = solution.Joints;
        }

        var forward = Enumerable.Range(0, divisions + 1).ToArray();
        var random = forward.ToArray();
        new Random(1920).Shuffle(random);

        int[][] orders = [forward, [.. Enumerable.Reverse(forward)], random];

        foreach (var order in orders)
        {
            foreach (int i in order)
            {
                // The final sample lies on the blend, not the original straight path.
                if (flyby && i == divisions)
                    continue;

                program.Animate(program.Targets[1].TotalTime * i / divisions, false);
                var actual = program.CurrentSimulationPose.Kinematics[0];
                Assert.That(actual.Errors, Is.Empty);
                Assert.That(actual.Joints, Is.EqualTo(expected[i]).Within(1e-6), $"Sample {i}");
            }
        }
    }

    [TestCase(5, 1)]
    [TestCase(1, 4)]
    public void LinearMotionReusesOnlyUnsubdividedEndpoints(double stepSize, int expectedCalls)
    {
        var system = TestRobots.AbbIrb120();
        var robot = system.GetRobot(0);
        double[] joints = [0.3, 1.1, 0.4, -0.5, 0.7, 0.6];
        Tool previousTool = new(Plane.WorldXY.WithOrigin(0, 0, 30));
        Tool tool = new(Plane.WorldXY.WithOrigin(10, 0, 40));
        Plane framePlane = new(new(10, 20, 30), Vector3d.YAxis, -Vector3d.XAxis);
        Frame frame = new(framePlane);
        JointTarget start = new(joints, tool: previousTool);
        Plane endWorld = system.Kinematics([new JointTarget(joints, tool: tool)])[0].Planes[^1];
        endWorld.OriginX += 2.5;
        Plane endPlane = endWorld;
        _ = endPlane.Transform(Transform.PlaneToPlane(framePlane, Plane.WorldXY));
        CartesianTarget end = new(endPlane, motion: Motions.Linear, tool: tool, frame: frame);
        CountingKinematics solver = new(robot);
        robot.Solver = solver;

        Program program = new("Samples", system, [TestRobots.Toolpath(start, end)], stepSize: stepSize);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(solver.PreviousJoints, Has.Count.EqualTo(expectedCalls));
            Assert.That(solver.PreviousJoints[0], Is.EqualTo(joints));
            Assert.That(program.Targets[^1].Planes[^1].Origin.DistanceTo(endWorld.Origin), Is.LessThan(1e-8));
            Assert.That(end.Plane, Is.EqualTo(endPlane), "The input target must remain unchanged.");

            if (expectedCalls > 1)
                Assert.That(solver.PreviousJoints[1], Is.EqualTo(joints).Within(1e-8));
        });
    }

    [TestCase(false)]
    [TestCase(true)]
    public void LinearMotionPreservesOtherGroupAndCoupledExternalEndpoint(bool coupled)
    {
        var system = TestRobots.AbbTwoGroupWithCustomExternal();
        double[] joints = [0.3, 1.1, 0.4, -0.5, 0.7, 0.6];
        double[] endJoints = [0.31, 1.1, 0.4, -0.5, 0.7, 0.6];
        JointTarget otherStart = new(joints, external: [0]);
        JointTarget otherEnd = new(joints, external: [5]);
        var expected = system.Kinematics([new JointTarget(endJoints), otherEnd]);
        Plane plane = expected[0].Planes[^1];
        Frame frame = new(Plane.WorldXY);

        if (coupled)
        {
            _ = plane.Transform(Transform.PlaneToPlane(expected[1].Planes[1], Plane.WorldXY));
            frame = new(Plane.WorldXY, coupledMechanism: 0, coupledMechanicalGroup: 1);
        }

        CartesianTarget end = new(plane, motion: Motions.Linear, frame: frame);
        Program program = new("Groups", system,
            [TestRobots.Toolpath(new JointTarget(joints), end), TestRobots.Toolpath(otherStart, otherEnd)], stepSize: 1);

        Assert.That(program.Errors, Is.Empty);
        var actual = program.Targets[^1].ProgramTargets;

        Assert.Multiple(() =>
        {
            Assert.That(actual[0].Kinematics.Joints, Is.EqualTo(expected[0].Joints).Within(1e-8));
            Assert.That(actual[1].Kinematics.Joints, Is.EqualTo(expected[1].Joints).Within(1e-8));
            Assert.That(actual[0].Kinematics.Planes[^1].Origin.DistanceTo(expected[0].Planes[^1].Origin), Is.LessThan(1e-8));
        });
    }

    [TestCase(-0.2)]
    [TestCase(-0.13)]
    public void PureRotationChecksIntermediateSingularities(double wrist)
    {
        var robot = TestRobots.AbbIrb120();
        Tool tool = new(Plane.WorldXY.WithOrigin(0, 0, -72));
        JointTarget start = new([0.3, 1.1, 0.4, -0.5, 0.2, 0.6], tool: tool);
        JointTarget endJoints = new([0.3, 1.1, 0.4, -0.5, wrist, 0.6], tool: tool);
        var endPlane = robot.Kinematics([endJoints])[0].Planes[^1];
        CartesianTarget end = new(endPlane, motion: Motions.Linear, tool: tool);

        Program program = new("Rotation", robot, [TestRobots.Toolpath(start, end)]);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Has.Some.Contains("singularity"));
            Assert.That(program.Code, Is.Null);
        });
    }

    [Test]
    public void LinearMotionChecksBranchChangesBetweenTranslationSamples()
    {
        CartesianTarget start = new(Plane.WorldZX.WithOrigin(500, 250, 600), RobotConfigurations.Wrist, Motions.Joint);
        CartesianTarget end = new(Plane.WorldZX.WithOrigin(500, 300, 600), motion: Motions.Linear);

        Program program = new("Wrist", TestRobots.UR10(), [TestRobots.Toolpath(start, end)], stepSize: 50);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Has.Some.Contains("singularity"));
            Assert.That(program.Code, Is.Null);
        });
    }

    [Test]
    public void RotationTimeUsesTheFullRelativeAngle()
    {
        var robot = TestRobots.AbbIrb120();
        Speed speed = new(rotationSpeed: 0.01);
        JointTarget start = new([0.3, 1.1, 0.4, -0.5, 0.7, 0.6], speed: speed);
        var startPlane = robot.Kinematics([start])[0].Planes[^1];
        var endPlane = startPlane;
        _ = endPlane.Rotate(0.2, startPlane.XAxis + startPlane.YAxis + startPlane.ZAxis, startPlane.Origin);
        CartesianTarget end = new(endPlane, motion: Motions.Linear, speed: speed);

        Program program = new("Rotation", robot, [TestRobots.Toolpath(start, end)]);
        program.Animate(10, false);
        var halfway = program.CurrentSimulationPose.GetLastPlane(0);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(20).Within(1e-10));
            Assert.That(GeometryMath.RotationAngle(startPlane, halfway), Is.EqualTo(0.1).Within(1e-10));
        });
    }

    [TestCase(0, false)]
    [TestCase(0, true)]
    [TestCase(1, false)]
    [TestCase(1, true)]
    public void WaitTimingRespectsItsTargetAndPhase(int target, bool runBefore)
    {
        var robot = TestRobots.AbbIrb120();
        double[][] joints = [[0.3, 1.1, 0.4, -0.5, 0.2, 0.6], [0.4, 1.1, 0.4, -0.5, 0.2, 0.6]];
        var targets = joints.Select((values, index) => new JointTarget(
            values, speed: new(time: 2), command: index == target ? new Wait(5, runBefore) : null)).ToArray();

        Program program = new("Waits", robot, [new SimpleToolpath(targets)]);
        bool waitFirst = target == 0 || runBefore;
        program.Animate(waitFirst ? 2.5 : 4.5, false);
        double held = program.CurrentSimulationPose.Kinematics[0].Joints[0];
        program.Animate(waitFirst ? 6 : 1, false);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(7).Within(1e-12));
            Assert.That(program.Targets[1].DeltaTime, Is.EqualTo(2).Within(1e-12));
            Assert.That(held, Is.EqualTo(waitFirst ? 0.3 : 0.4).Within(1e-12));
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints[0], Is.EqualTo(0.35).Within(1e-12));
        });
    }

    [TestCase(0, false)]
    [TestCase(1, true)]
    [TestCase(1, false)]
    [TestCase(3, true)]
    [TestCase(3, false)]
    public void FlyByReconstructionPreservesWaits(int target, bool runBefore)
    {
        var robot = TestRobots.AbbIrb120();
        var targets = Enumerable.Range(0, 4).Select(index => new JointTarget(
            [0.3 + index * 0.1, 1.1, 0.4, -0.5, 0.2, 0.6],
            speed: new(time: 2), zone: new(1), command: index == target ? new Wait(5, runBefore) : null)).ToArray();

        Program program = new("Waits", robot, [new SimpleToolpath(targets)]);
        double waitStart = 2 * Math.Max(0, target - (runBefore ? 1 : 0));
        program.Animate(waitStart + 2.5, false);
        double expected = 0.3 + Math.Max(0, target - (runBefore ? 1 : 0)) * 0.1;

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(11).Within(1e-12));
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints[0], Is.EqualTo(expected).Within(1e-12));
            Assert.That(program.MotionSegments.Any(segment => segment.Corner is not null), Is.True);
        });
    }

    [Test]
    public void SingleTargetWaitHasAStationarySimulation()
    {
        JointTarget target = new([0.3, 1.1, 0.4, -0.5, 0.2, 0.6], command: new Wait(5));
        Program program = new("Waits", TestRobots.AbbIrb120(), [target]);
        program.Animate(0.5);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Duration, Is.EqualTo(5));
            Assert.That(program.CurrentSimulationPose.Kinematics[0].Joints, Is.EqualTo(target.Joints));
        });
    }

    class CountingKinematics(RobotArm robot) : SphericalWristKinematics(robot)
    {
        internal List<double[]> PreviousJoints { get; } = [];

        protected override void SetJoints(KinematicSolution solution, Target target, PreviousJoints prevJoints)
        {
            if (target is CartesianTarget)
                PreviousJoints.Add(prevJoints.Values.ToArray());

            base.SetJoints(solution, target, prevJoints);
        }
    }

    class ErrorRegionKinematics(RobotArm robot, Point3d point) : SphericalWristKinematics(robot)
    {
        protected override void SetJoints(KinematicSolution solution, Target target, PreviousJoints prevJoints)
        {
            base.SetJoints(solution, target, prevJoints);

            if (target is CartesianTarget cartesian && cartesian.Plane.Origin.DistanceTo(point) < 1e-6)
                solution.AddError("Test blend failure");
        }
    }
}
