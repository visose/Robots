using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

class FixedRedundancyKinematicsTests
{
    const double AngleTolerance = 2e-5;
    const double PositionTolerance = 1e-5;
    const double OrientationTolerance = 1e-8;

    [Test]
    public void SolverSelectionUsesGeometry()
    {
        var supported = CreateRobot("UnrelatedModel", terminalOffset: 88);
        var supportedWithStartPose = CreateRobot("OtherModel", terminalOffset: 88, startTheta: 30);
        var unsupported = CreateRobot("Panda", terminalOffset: 0);

        Assert.Multiple(() =>
        {
            Assert.That(FixedRedundancyKinematics.Supports(supported), Is.True);
            Assert.That(supported.Solver, Is.TypeOf<FixedRedundancyKinematics>());
            Assert.That(supported.Solver.RedundantJointIndex, Is.EqualTo(2));
            Assert.That(FixedRedundancyKinematics.Supports(supportedWithStartPose), Is.True);
            Assert.That(supportedWithStartPose.Solver, Is.TypeOf<FixedRedundancyKinematics>());
            Assert.That(FixedRedundancyKinematics.Supports(unsupported), Is.False);
            Assert.That(unsupported.Solver, Is.TypeOf<NumericalKinematics>());
            Assert.That(unsupported.Solver.RedundantJointIndex, Is.EqualTo(2));
        });
    }

    [Test]
    public void RandomPosesRecoverOriginalAndValidateEveryBranch()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        var random = new Random(191_307);

        for (int sample = 0; sample < 256; sample++)
        {
            var joints = RandomJoints(robot, random);
            AssertRoundTrip(robot, solver, joints, $"Random sample {sample}");
        }
    }

    [Test]
    public void RedundantJointUsesExternalThenPreviousThenMidpoint()
    {
        var robot = GetRobot();
        double midpoint = robot.Joints[2].Range.Mid;
        double[] externalJoints = [0.25, -0.65, 0.45, -1.35, 0.8, 1.1, -0.55];
        double[] externalPrevious = [.. externalJoints];
        externalPrevious[2] = -0.35;
        double[] previousJoints = [-0.35, -0.55, -0.4, -1.15, 0.65, 1.35, 0.3];
        double[] midpointJoints = [0.4, -0.7, midpoint, -1.4, 0.75, 1.2, -0.5];

        Assert.Multiple(() =>
        {
            AssertRedundancy(externalJoints, externalPrevious, [externalJoints[2]], "External source");
            AssertRedundancy(previousJoints, previousJoints, null, "Previous source");
            AssertRedundancy(midpointJoints, null, null, "Midpoint source");
        });

        void AssertRedundancy(double[] joints, double[]? previous, double[]? external, string source)
        {
            var solution = robot.Kinematics(CartesianTarget(robot, joints, external), previous);
            Assert.That(solution.Errors, Is.Empty, source);
            Assert.That(solution.Joints[2], Is.EqualTo(joints[2]).Within(1e-10), source);
        }
    }

    [Test]
    public void InitialCartesianTargetUsesMidpointRedundancy()
    {
        var system = TestRobots.FrankaPanda();
        var robot = ((SingleGroupSystem)system).Robot;
        double[] joints = [0.25, -0.65, robot.Joints[2].Range.Mid, -1.35, 0.8, 1.1, -0.55];
        var plane = robot.Kinematics(new JointTarget(joints)).Planes[^1];
        var target = new CartesianTarget(plane, motion: Motions.Linear);
        var program = new Program("P", system, [TestRobots.Toolpath(target)], stepSize: 1000);

        Assert.Multiple(() =>
        {
            Assert.That(program.Errors, Is.Empty);
            Assert.That(program.Code, Is.Not.Null);
            Assert.That(program.Warnings, Has.Some.Contains("First target changed to a joint target."));
            Assert.That(system.PostProcessor, Is.TypeOf<FrankxPostProcessor>());
        });
    }

    [Test]
    public void ExplicitRedundantJointIsNotRewoundIntoRange()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        double[] joints = [0.25, -0.65, 0.45, -1.35, 0.8, 1.1, -0.55];
        var target = Forward(robot, joints);
        var solutions = solver.GetSolutions(
            target,
            joints[2] + 2 * Math.PI,
            previous: null,
            out var errors);

        Assert.Multiple(() =>
        {
            Assert.That(solutions, Is.Empty);
            Assert.That(errors, Does.Contain("Target requires joints outside the permitted ranges."));
        });
    }

    [Test]
    public void PreviousJointsMaintainTheNearestBranch()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        double[] source = [0.45, -0.75, 0.3, -1.4, 0.9, 1.2, -0.6];
        var transform = Forward(robot, source);
        var branches = solver.GetSolutions(transform, source[2], previous: null, out var errors);

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty);
            Assert.That(branches.Count, Is.GreaterThan(1));
        });

        var preferred = branches[^1];
        var previous = NearbyJoints(robot, preferred);
        var target = CartesianTarget(robot, preferred, [preferred[2]]);
        var selected = robot.Kinematics(target, previous);
        var nearest = branches.MinBy(branch => SquaredDifference(branch, previous));

        Assert.Multiple(() =>
        {
            Assert.That(selected.Errors, Is.Empty);
            Assert.That(nearest, Is.Not.Null);
            Assert.That(SameAngles(selected.Joints, nearest!), Is.True, "Nearest branch was not selected.");
        });

        double[] next = [.. selected.Joints];
        next[0] = MoveInsideRange(robot.Joints[0].Range, next[0], 0.01);
        next[3] = MoveInsideRange(robot.Joints[3].Range, next[3], -0.01);
        var continued = robot.Kinematics(CartesianTarget(robot, next, [next[2]]), selected.Joints);

        Assert.Multiple(() =>
        {
            Assert.That(continued.Errors, Is.Empty);
            Assert.That(SameAngles(continued.Joints, next), Is.True);
            Assert.That(MaximumDifference(continued.Joints, selected.Joints), Is.LessThan(0.1));
        });
    }

    [Test]
    public void JointBoundariesAndNearSingularPosesRoundTrip()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        double[] regular = [0.35, -0.55, 0.4, -1.2, 0.7, 1.1, -0.45];
        var cases = new (string Name, double[] Joints)[]
        {
            ("Axis 1 lower boundary", With(regular, 0, robot.Joints[0].Range.T0 + 1e-5)),
            ("Redundant axis upper boundary", With(regular, 2, robot.Joints[2].Range.T1 - 1e-5)),
            ("Axis 4 upper boundary", With(regular, 3, robot.Joints[3].Range.T1 - 1e-5)),
            ("Half-angle pole", With(regular, 5, Math.PI)),
            ("Near flat shoulder", With(regular, 1, 1e-5)),
            ("Near intersecting-axis singularity", With(regular, 4, 1e-5))
        };

        foreach (var (name, joints) in cases)
            AssertRoundTrip(robot, solver, joints, name);
    }

    [Test]
    public void ExactJointBoundariesRoundTrip()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        double[] regular = [0.35, -0.55, 0.4, -1.2, 0.7, 1.1, -0.45];

        for (int joint = 0; joint < robot.Joints.Length; joint++)
        {
            var range = robot.Joints[joint].Range;
            AssertRoundTrip(robot, solver, With(regular, joint, range.T0), $"Joint {joint + 1} lower boundary");
            AssertRoundTrip(robot, solver, With(regular, joint, range.T1), $"Joint {joint + 1} upper boundary");
        }
    }

    [TestCase(0)]
    [TestCase(1e-12)]
    [TestCase(1e-10)]
    [TestCase(1e-9)]
    [TestCase(1.5e-9)]
    [TestCase(2e-9)]
    [TestCase(2.3e-9)]
    [TestCase(2.4e-9)]
    [TestCase(5e-9)]
    [TestCase(7.5e-9)]
    [TestCase(7.9e-9)]
    [TestCase(8e-9)]
    [TestCase(8.5e-9)]
    [TestCase(1e-8)]
    [TestCase(1e-7)]
    [TestCase(1e-6)]
    public void NearParallelReducedAxesRoundTrip(double offset)
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        double[] regular = [0.35, -0.55, 0, -1.2, 0.7, 1.1, -0.45];
        AssertRoundTrip(robot, solver, With(regular, 2, offset), $"Redundant joint {offset:G3}");

        if (offset != 0)
            AssertRoundTrip(robot, solver, With(regular, 2, -offset), $"Redundant joint {-offset:G3}");
    }

    [Test]
    public void NearParallelReducedAxesRecoverRandomPoses()
    {
        ReadOnlySpan<double> offsets = [5e-9, 8e-9, 8.2e-9, 1e-8, 2e-8];
        var robot = GetRobot();
        var solver = GetSolver(robot);
        var random = new Random(472_391);

        foreach (double magnitude in offsets)
        {
            for (int sign = -1; sign <= 1; sign += 2)
            {
                double offset = sign * magnitude;

                for (int sample = 0; sample < 32; sample++)
                {
                    var joints = RandomJoints(robot, random);
                    joints[2] = offset;
                    AssertRoundTrip(
                        robot,
                        solver,
                        joints,
                        $"Redundant joint {offset:G3}, sample {sample}");
                }
            }
        }
    }

    [Test]
    public void NearParallelReducedAxesRecoverFormerlyOmittedBranches()
    {
        double[][] cases =
        [
            [1.98494410639859, -1.0203432760528957, -5e-8, -1.727841135965586, 1.2037428373953993, 1.6286440066893788, -1.4774328383512016],
            [-0.622421229264146, -0.7070676017065922, -5e-8, -1.2430222788931231, -1.5282984055348818, 1.568736640511061, 0.14191886474473603],
            [-1.1581374877738613, -0.8780814265350043, 5e-8, -1.8584631654780903, 0.005357377024068211, 2.479517824199638, -1.3686395686141508],
            [0.548394783488352, -0.6218491784906528, 5e-8, -1.1603337501173019, -0.026072332112152288, 1.228256534935021, 0.26074723677531253],
            [1.2460848712079189, -0.319565067343198, 5e-8, -0.7287624995987259, 0.006487423300970074, 2.7195768421800866, -1.543691414578571]
        ];
        var robot = GetRobot();
        var solver = GetSolver(robot);

        for (int i = 0; i < cases.Length; i++)
            AssertRoundTrip(robot, solver, cases[i], $"Formerly omitted branch {i}");
    }

    [Test]
    public void ExactSingularPosesReturnValidBranches()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        double[] regular = [0.35, -0.55, 0.4, -1.2, 0.7, 1.1, -0.45];
        double[] setA = [.. regular];
        setA[1] = 0;
        setA[2] = Math.PI / 2;
        setA[4] = Math.PI / 2;
        double[] setC = [.. regular];
        double a5 = robot.Joints[4].A;
        double d3 = robot.Joints[2].D;
        double d5 = robot.Joints[4].D;
        setC[3] = Math.Atan(a5 * (d3 + d5) / (-a5 * a5 + d5 * d3));
        setC[4] = 0;
        double[] setCChartBoundary = [.. setC];
        setCChartBoundary[0] = 0;
        var cases = new (string Name, double[] Joints)[]
        {
            ("Singular set A", setA),
            ("Singular set C", setC),
            ("Singular set C at chart boundary", setCChartBoundary)
        };

        foreach (var (name, joints) in cases)
        {
            var target = Forward(robot, joints);
            var solutions = solver.GetSolutions(target, joints[2], previous: null, out var errors);

            Assert.Multiple(() =>
            {
                Assert.That(errors, Is.Empty, name);
                Assert.That(solutions, Is.Not.Empty, name);
            });
            AssertAllSolutions(robot, target, joints[2], solutions, name);
            var selected = robot.Kinematics(
                CartesianTarget(robot, joints, [joints[2]]),
                joints);

            Assert.Multiple(() =>
            {
                Assert.That(selected.Errors, Does.Contain("Target near singularity."), name);
                Assert.That(SameAngles(selected.Joints, joints), Is.True, $"{name}: previous branch");
            });
        }
    }

    [Test]
    public void UnreachableTargetReturnsNoBranch()
    {
        var robot = GetRobot();
        var solver = GetSolver(robot);
        var target = Transform.Identity;
        target.M03 = 10_000;
        var solutions = solver.GetSolutions(
            target,
            robot.Joints[2].Range.Mid,
            previous: null,
            out var errors);

        Assert.Multiple(() =>
        {
            Assert.That(solutions, Is.Empty);
            Assert.That(errors, Does.Contain("Target out of reach."));
        });
    }

    [Test]
    public void SolveAllocationsStayBounded()
    {
        const int allocationLimit = 8 * 1024;
        var robot = GetRobot();
        double[] joints = [0.2, -0.5, 0.4, -1.2, 0.7, 1.0, -0.4];
        var target = CartesianTarget(robot, joints, [joints[2]]);

        for (int i = 0; i < 8; i++)
            _ = robot.Kinematics(target, joints);

        long before = GC.GetAllocatedBytesForCurrentThread();
        var solution = robot.Kinematics(target, joints);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        TestContext.Out.WriteLine($"Fixed-redundancy solve allocated {allocated / 1024.0:0.###} KiB.");

        Assert.Multiple(() =>
        {
            Assert.That(solution.Errors, Is.Empty);
            Assert.That(allocated, Is.LessThan(allocationLimit));
        });
    }

    [Test]
    [Explicit("Reports fixed-redundancy analytical and numerical solve performance.")]
    public void BenchmarkAgainstNumericalSolver()
    {
        const int count = 64;
        const int warmupCount = 256;
        var robot = GetRobot();
        double[] joints = [0.2, -0.5, 0.4, -1.2, 0.7, 1.0, -0.4];
        var target = CartesianTarget(robot, joints, [joints[2]]);
        double[] nearby = [0.28, -0.58, joints[2], -1.12, 0.62, 1.08, -0.32];
        var numerical = new NumericalKinematics(robot, useModifiedDH: true, redundant: 2);
        Report("analytical nearby", robot.Solver, new(nearby));
        Report("numerical nearby", numerical, new(nearby));
        Report("analytical unseeded", robot.Solver, default);
        Report("numerical unseeded", numerical, default);

        void Report(
            string name,
            MechanismKinematics solver,
            PreviousJoints previous)
        {
            for (int i = 0; i < warmupCount; i++)
                _ = solver.Solve(target, previous, basePlane: null);

            long allocated = 0;
            long start = System.Diagnostics.Stopwatch.GetTimestamp();
            bool successful = true;

            for (int i = 0; i < count; i++)
            {
                long before = GC.GetAllocatedBytesForCurrentThread();
                var solution = solver.Solve(target, previous, basePlane: null);
                allocated += GC.GetAllocatedBytesForCurrentThread() - before;
                successful &= solution.Errors.Count == 0;
            }

            double microseconds = System.Diagnostics.Stopwatch.GetElapsedTime(start).TotalMicroseconds / count;
            TestContext.Out.WriteLine(
                $"{name}: {microseconds:0.###} us/solve, {allocated / (1024.0 * count):0.###} KiB/solve");
            Assert.That(successful, Is.True, name);
        }
    }

    static void AssertRoundTrip(
        RobotArm robot,
        FixedRedundancyKinematics solver,
        double[] joints,
        string message)
    {
        var target = Forward(robot, joints);
        var solutions = solver.GetSolutions(target, joints[2], previous: null, out var errors);

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty, message);
            Assert.That(solutions, Is.Not.Empty, message);
            Assert.That(
                solutions.Any(solution => SameAngles(solution, joints)),
                Is.True,
                $"{message}: original branch was not recovered.");
        });

        AssertAllSolutions(robot, target, joints[2], solutions, message);
    }

    static void AssertAllSolutions(
        RobotArm robot,
        Transform target,
        double redundant,
        List<double[]> solutions,
        string message)
    {
        for (int solutionIndex = 0; solutionIndex < solutions.Count; solutionIndex++)
        {
            var solution = solutions[solutionIndex];
            var actual = Forward(robot, solution);
            var (positionError, orientationError) = PoseError(target, actual);

            Assert.Multiple(() =>
            {
                Assert.That(
                    positionError,
                    Is.LessThan(PositionTolerance),
                    $"{message}, branch {solutionIndex}: position");
                Assert.That(
                    orientationError,
                    Is.LessThan(OrientationTolerance),
                    $"{message}, branch {solutionIndex}: orientation");
                Assert.That(
                    Math.Abs(Math.IEEERemainder(solution[2] - redundant, 2 * Math.PI)),
                    Is.LessThan(1e-10),
                    $"{message}, branch {solutionIndex}: fixed joint");

                for (int jointIndex = 0; jointIndex < solution.Length; jointIndex++)
                {
                    Assert.That(
                        solution[jointIndex],
                        Is.InRange(
                            robot.Joints[jointIndex].Range.T0 - 1e-10,
                            robot.Joints[jointIndex].Range.T1 + 1e-10),
                        $"{message}, branch {solutionIndex}, joint {jointIndex + 1}");
                }
            });
        }

        for (int first = 0; first < solutions.Count - 1; first++)
        {
            for (int second = first + 1; second < solutions.Count; second++)
            {
                Assert.That(
                    SameAngles(solutions[first], solutions[second], 1e-7),
                    Is.False,
                    $"{message}: duplicate branches {first} and {second}.");
            }
        }
    }

    static double[] RandomJoints(RobotArm robot, Random random)
    {
        var joints = new double[robot.Joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            var range = robot.Joints[i].Range;
            double margin = (range.T1 - range.T0) * 0.18;
            joints[i] = range.T0 + margin
                + random.NextDouble() * (range.T1 - range.T0 - margin * 2);
        }

        if (Math.Abs(joints[1]) < 0.25)
            joints[1] = Math.CopySign(0.25, joints[1] == 0 ? 1 : joints[1]);

        if (Math.Abs(Math.Sin(joints[4])) < 0.2)
            joints[4] = MoveInsideRange(robot.Joints[4].Range, joints[4], 0.35);

        return joints;
    }

    static double[] NearbyJoints(RobotArm robot, double[] joints)
    {
        var nearby = new double[joints.Length];

        for (int i = 0; i < nearby.Length; i++)
        {
            double offset = i == 2 ? 0 : (i & 1) == 0 ? 0.015 : -0.015;
            nearby[i] = MoveInsideRange(robot.Joints[i].Range, joints[i], offset);
        }

        return nearby;
    }

    static double MoveInsideRange(Interval range, double value, double offset)
    {
        const double margin = 1e-6;
        return Math.Clamp(value + offset, range.T0 + margin, range.T1 - margin);
    }

    static double[] With(double[] source, int index, double value)
    {
        double[] result = [.. source];
        result[index] = value;
        return result;
    }

    static Transform Forward(RobotArm robot, double[] joints)
    {
        var transform = Transform.Identity;

        for (int i = 0; i < joints.Length; i++)
        {
            var definition = robot.Joints[i];
            double cosine = Math.Cos(joints[i]);
            double sine = Math.Sin(joints[i]);
            double cosineAlpha = Math.Cos(definition.Alpha);
            double sineAlpha = Math.Sin(definition.Alpha);
            Transform current = default;
            current.Set(
                cosine, -sine, 0, definition.A,
                sine * cosineAlpha, cosine * cosineAlpha, -sineAlpha, -definition.D * sineAlpha,
                sine * sineAlpha, cosine * sineAlpha, cosineAlpha, definition.D * cosineAlpha);
            transform *= current;
        }

        return transform;
    }

    static CartesianTarget CartesianTarget(
        RobotArm robot,
        double[] joints,
        double[]? external = null)
    {
        var plane = robot.Kinematics(new JointTarget(joints)).Planes[^1];
        return new(plane, motion: Motions.Joint, external: external);
    }

    static (double Position, double Orientation) PoseError(
        Transform expected,
        Transform actual)
    {
        double dx = actual.M03 - expected.M03;
        double dy = actual.M13 - expected.M13;
        double dz = actual.M23 - expected.M23;
        double position = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        double orientation = 0;

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                orientation = Math.Max(
                    orientation,
                    Math.Abs(actual[row, column] - expected[row, column]));
            }
        }

        return (position, orientation);
    }

    static bool SameAngles(double[] first, double[] second, double tolerance = AngleTolerance)
    {
        for (int i = 0; i < first.Length; i++)
        {
            if (Math.Abs(Math.IEEERemainder(first[i] - second[i], 2 * Math.PI)) > tolerance)
                return false;
        }

        return true;
    }

    static double SquaredDifference(double[] first, double[] second)
    {
        double difference = 0;

        for (int i = 0; i < first.Length; i++)
        {
            double jointDifference = first[i] - second[i];
            difference += jointDifference * jointDifference;
        }

        return difference;
    }

    static double MaximumDifference(double[] first, double[] second)
    {
        double difference = 0;

        for (int i = 0; i < first.Length; i++)
            difference = Math.Max(difference, Math.Abs(first[i] - second[i]));

        return difference;
    }

    static FixedRedundancyKinematics GetSolver(RobotArm robot) =>
        (FixedRedundancyKinematics)robot.Solver;

    static RobotArm GetRobot() => ((SingleGroupSystem)TestRobots.FrankaPanda()).Robot;

    static RobotArm CreateRobot(string model, double terminalOffset, double? startTheta = null)
    {
        string startThetaAttribute = startTheta is null ? "" : $" θ=\"{startTheta}\"";
        var xml = $"""
            <RobotSystem name="FixedRedundancyTest" manufacturer="FrankaEmika">
              <Mechanisms>
                <RobotArm model="{model}" manufacturer="FrankaEmika" payload="3">
                  <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
                  <Joints>
                    <Revolute number="1" a="0" d="333" minrange="-166" maxrange="166" maxspeed="150"{startThetaAttribute}/>
                    <Revolute number="2" a="0" d="0" minrange="-101" maxrange="101" maxspeed="150"/>
                    <Revolute number="3" a="0" d="316" minrange="-166" maxrange="166" maxspeed="150"/>
                    <Revolute number="4" a="82.5" d="0" minrange="-176" maxrange="-4" maxspeed="150"/>
                    <Revolute number="5" a="-82.5" d="384" minrange="-166" maxrange="166" maxspeed="180"/>
                    <Revolute number="6" a="0" d="0" minrange="-1" maxrange="215" maxspeed="180"/>
                    <Revolute number="7" a="{terminalOffset}" d="107" minrange="-166" maxrange="166" maxspeed="180"/>
                  </Joints>
                </RobotArm>
              </Mechanisms>
            </RobotSystem>
            """;

        return ((SingleGroupSystem)FileIO.ParseRobotSystem(xml, Plane.WorldXY)).Robot;
    }
}
