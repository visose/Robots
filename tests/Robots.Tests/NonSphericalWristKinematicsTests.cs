using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

class NonSphericalWristKinematicsTests
{
    static readonly (Func<RobotSystem> Factory, int Seed, double SingularJoint5)[] Cases =
    [
        (TestRobots.AbbPowa1920, 1920, 0.0062625742039908085),
        (TestRobots.AbbGofa10, 15010, 0.07556903629345449),
        (TestRobots.AbbGofa12, 15012, 0.09146244096169531)
    ];

    [Test]
    public void AbbSolverSelectionUsesGeometry()
    {
        var spherical = GetRobot(TestRobots.AbbIrb120());
        var unsupported = GetRobot(TestRobots.AbbNumerical());

        Assert.Multiple(() =>
        {
            Assert.That(spherical.Solver, Is.TypeOf<SphericalWristKinematics>());
            Assert.That(SphericalWristKinematics.Supports(spherical), Is.True);
            Assert.That(NonSphericalWristKinematics.Supports(spherical), Is.False);

            foreach (var (factory, _, _) in Cases)
            {
                var robot = GetRobot(factory());
                Assert.That(robot.Solver, Is.TypeOf<NonSphericalWristKinematics>(), robot.Model);
                Assert.That(NonSphericalWristKinematics.Supports(robot), Is.True, robot.Model);
                Assert.That(SphericalWristKinematics.Supports(robot), Is.False, robot.Model);
                Assert.That(robot.Solver.RequiresContinuation, Is.False, robot.Model);
            }

            Assert.That(unsupported.Solver, Is.TypeOf<NumericalKinematics>());
            Assert.That(SupportsAfter(joints => joints[0].Alpha += 5e-4), Is.False, "axis twist");
            Assert.That(SupportsAfter(joints => joints[1].D = 1), Is.False, "d2");
            Assert.That(SupportsAfter(joints => joints[3].A = 1), Is.False, "a4");
            Assert.That(SupportsAfter(joints => joints[5].A = 1), Is.False, "a6");
            Assert.That(SupportsAfter(joints => joints[1].A = 0), Is.False, "upper arm");
            Assert.That(SupportsAfter(joints =>
            {
                joints[2].A = 0;
                joints[3].D = 0;
            }), Is.False, "forearm");

            Assert.That(SupportsAfter(joints =>
            {
                joints[4].A = 0;
                joints[4].D = 0;
            }), Is.False, "final offset");

            Assert.That(SupportsAfter(joints => joints[0].D = double.NaN), Is.False, "finite DH");
        });
    }

    [Test]
    public void RandomPosesValidateEveryBranch()
    {
        foreach (var (factory, seed, _) in Cases)
        {
            var robot = GetRobot(factory());
            var solver = (NonSphericalWristKinematics)robot.Solver;
            var random = new Random(seed);

            for (int sample = 0; sample < 24; sample++)
                _ = AssertRoundTrip(robot, solver, RandomJoints(robot, random), previous: null, $"{robot.Model}, sample {sample}");
        }
    }

    [Test]
    public void PoWaAxis4BoundaryPathMaintainsConfiguration()
    {
        var robot = GetRobot(TestRobots.AbbPowa1920());
        JointTarget home = new([0, Math.PI * 0.5, 0, 0, 0, 0]);
        CartesianTarget approach = new(PathPlane(800, -300, 800), motion: Motions.Joint);
        var previous = robot.Kinematics(home);
        previous = robot.Kinematics(approach, previous.Joints);
        var configuration = previous.Configuration;
        CartesianTarget start = new(PathPlane(800, -300, 700), configuration, Motions.Linear);
        previous = robot.Kinematics(start, previous.Joints);

        Assert.That(previous.Errors, Is.Empty);

        for (int x = 801; x <= 1050; x++)
        {
            CartesianTarget target = new(PathPlane(x, -300, 700), configuration, Motions.Linear);
            var current = robot.Kinematics(target, previous.Joints);

            Assert.Multiple(() =>
            {
                Assert.That(current.Errors, Is.Empty, $"X={x}");
                Assert.That(current.Configuration, Is.EqualTo(configuration), $"X={x}");
            });

            previous = current;
        }
    }

    [Test]
    public void Axis4ContinuumEnumeratesEverySeededBranch()
    {
        var robot = GetRobot(TestRobots.AbbPowa1920());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        JointTarget home = new([0, Math.PI * 0.5, 0, 0, 0, 0]);
        CartesianTarget approach = new(PathPlane(800, -300, 800), motion: Motions.Joint);
        var previous = robot.Kinematics(home);
        previous = robot.Kinematics(approach, previous.Joints);
        CartesianTarget start = new(
            PathPlane(800, -300, 700),
            previous.Configuration,
            Motions.Linear);

        previous = robot.Kinematics(start, previous.Joints);

        for (int x = 801; x <= 850; x++)
        {
            CartesianTarget target = new(
                PathPlane(x, -300, 700),
                previous.Configuration,
                Motions.Linear);

            previous = robot.Kinematics(target, previous.Joints);
        }

        var targetTransform = Forward(robot, previous.Joints);

        double orientationNorm = Math.Sqrt(
            targetTransform.M20 * targetTransform.M20
            + targetTransform.M21 * targetTransform.M21);

        Assert.That(orientationNorm, Is.LessThan(1e-8));

        var independent = solver.GetSolutions(targetTransform, previous: null, out var independentErrors);
        var seededBranches = new List<WristSolution>();

        for (int seedIndex = 0; seedIndex < 48; seedIndex++)
        {
            double[] seed = [.. previous.Joints];
            seed[5] = -Math.PI + seedIndex * Math.PI / 24;
            var seeded = solver.GetSolutions(targetTransform, seed, out var errors);

            Assert.That(errors, Is.Empty, $"Seed {seedIndex}");

            foreach (var solution in seeded)
            {
                if (!seededBranches.Any(branch => SamePoseJoints(branch.Joints, solution.Joints, 1e-5)))
                    seededBranches.Add(solution);
            }
        }

        Assert.Multiple(() =>
        {
            Assert.That(independentErrors, Is.Empty);
            Assert.That(independent, Is.Not.Empty);

            foreach (var branch in seededBranches)
            {
                Assert.That(
                    independent.Any(solution => SamePoseJoints(solution.Joints, branch.Joints, 1e-5)),
                    Is.True,
                    $"Missing branch [{string.Join(", ", branch.Joints)}].");
            }
        });

        AssertAllSolutions(robot, targetTransform, independent);
    }

    [Test]
    [Explicit("Runs high-count full-range, boundary, singular, and selection validation for every supported ABB geometry.")]
    public void ExhaustiveStress()
    {
        ReadOnlySpan<double> singularOffsets =
        [
            0,
            -1e-12, 1e-12,
            -1e-10, 1e-10,
            -1e-9, 1e-9,
            -1e-8, 1e-8,
            -1e-6, 1e-6,
            -1e-5, 1e-5,
            -1e-4, 1e-4
        ];
        int sources = 0;
        int branches = 0;

        foreach (var (factory, baseSeed, singularJoint5) in Cases)
        {
            int seed = baseSeed * 1000 + 1;
            var system = factory();
            var robot = GetRobot(system);
            var solver = (NonSphericalWristKinematics)robot.Solver;
            var random = new Random(seed);

            for (int sample = 0; sample < 4096; sample++)
            {
                var joints = FullRangeJoints(robot, random);
                branches += AssertRoundTrip(
                    robot,
                    solver,
                    joints,
                    previous: null,
                    $"{robot.Model}, full-range sample {sample}",
                    sourceTolerance: 2e-5).Count;
                sources++;
            }

            var nominal = RandomJoints(robot, random);

            for (int jointIndex = 0; jointIndex < robot.Joints.Length; jointIndex++)
            {
                var range = robot.Joints[jointIndex].Range;

                foreach (double inset in new[] { 0.0, 1e-9 })
                {
                    foreach (bool upper in new[] { false, true })
                    {
                        double[] joints = [.. nominal];
                        joints[jointIndex] = upper ? range.T1 - inset : range.T0 + inset;
                        branches += AssertRoundTrip(
                            robot,
                            solver,
                            joints,
                            previous: null,
                            $"{robot.Model}, joint {jointIndex + 1} {(upper ? "upper" : "lower")} boundary, inset {inset:G3}",
                            sourceTolerance: 2e-5).Count;
                        sources++;
                    }
                }
            }

            foreach (double offset in singularOffsets)
            {
                double[] joints = [0.2, 0.4, 0.5, -0.6, singularJoint5 + offset, 0.7];
                branches += AssertRoundTrip(
                    robot,
                    solver,
                    joints,
                    joints,
                    $"{robot.Model}, singular offset {offset:G3}",
                    sourceTolerance: 2e-5).Count;
                sources++;
            }

            branches += AssertCompactSelection(system, seed + 1, 64);
            sources += 64;
        }

        TestContext.Out.WriteLine($"ABB non-spherical stress: {sources:N0} sources, {branches:N0} returned branches.");
        Assert.That(sources, Is.EqualTo(12_597));
    }

    [Test]
    public void ChartPolesRemainSolvable()
    {
        var robot = GetRobot(TestRobots.AbbPowa1920());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] joints = [0.3, 0.4, 0.5, -0.6, 0, Math.PI];
        var solutions = AssertRoundTrip(robot, solver, joints, joints);
        Assert.That(solutions.Where(solution => SamePoseJoints(solution.Joints, joints))
            .All(solution => !solution.IsNearSingular), Is.True);
    }

    [Test]
    public void NearBranchBoundariesRoundTrip()
    {
        var robot = GetRobot(TestRobots.AbbPowa1920());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double bendBoundary = Math.Atan2(robot.Joints[3].D, robot.Joints[2].A);
        double[] joints = [0.2, 0.3, bendBoundary + 1e-6, -0.4, 1e-6, 0.7];
        _ = AssertRoundTrip(robot, solver, joints, joints, sourceTolerance: 2e-5);
    }

    [Test]
    public void ElbowBoundaryIsNotSingular()
    {
        var system = TestRobots.AbbPowa1920();
        var robot = GetRobot(system);
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double bendBoundary = Math.Atan2(robot.Joints[3].D, robot.Joints[2].A);
        double[] joints = [0.2, 0.3, bendBoundary, -0.4, 0.35, 0.7];
        var target = Forward(robot, joints);
        var solutions = solver.GetSolutions(target, joints, out var errors);
        var independent = solver.GetSolutions(target, previous: null, out var independentErrors);
        var flange = system.Kinematics([new JointTarget(joints)])[0].Planes[^1];
        var cartesian = new CartesianTarget(flange, motion: Motions.Joint);
        var publicSolution = system.Kinematics([cartesian], [joints])[0];

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty);
            Assert.That(solutions.Any(solution => SamePoseJoints(solution.Joints, joints, 2e-5)), Is.True);
            Assert.That(
                solutions.Where(solution => SamePoseJoints(solution.Joints, joints, 2e-5))
                    .All(solution => !solution.IsNearSingular),
                Is.True);

            Assert.That(independentErrors, Is.Empty);
            Assert.That(independent, Is.Not.Empty);
            Assert.That(publicSolution.Errors, Does.Not.Contain("Target near singularity."));
        });

        AssertAllSolutions(robot, target, solutions);
        AssertAllSolutions(robot, target, independent);
    }

    [Test]
    public void ShoulderContinuumUsesPreviousJoint()
    {
        var robot = GetRobot(TestRobots.AbbGofa12());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double joint2 = Math.Acos(robot.Joints[2].A / robot.Joints[1].A);
        double[] joints = [0.4, joint2, Math.PI - joint2, -0.2, 0.5, 0.6];
        var solutions = AssertRoundTrip(robot, solver, joints, joints, sourceTolerance: 2e-5);
        Assert.That(solutions.Where(solution => SamePoseJoints(solution.Joints, joints, 2e-5))
            .All(solution => solution.IsNearSingular), Is.True);
    }

    [Test]
    public void SingularitiesAreReported()
    {
        foreach (var (factory, _, joint5) in Cases)
        {
            var system = factory();
            var robot = GetRobot(system);
            var solver = (NonSphericalWristKinematics)robot.Solver;
            double[] singular = [0.2, 0.4, 0.5, -0.6, joint5, 0.7];

            AssertSingularity(singular, expected: true, "singular");

            double[] near = [.. singular];
            near[4] += 1e-5;
            AssertSingularity(near, expected: true, "near singular");

            double[] regular = [.. singular];
            regular[4] += 1e-4;
            AssertSingularity(regular, expected: false, "regular");

            void AssertSingularity(double[] joints, bool expected, string state)
            {
                string message = $"{robot.Model}, {state}";
                var solutions = AssertRoundTrip(robot, solver, joints, joints, message, sourceTolerance: 2e-5);
                var matching = solutions
                    .Where(solution => SamePoseJoints(solution.Joints, joints, 2e-5))
                    .ToArray();

                var flange = system.Kinematics([new JointTarget(joints)])[0].Planes[^1];
                var cartesian = new CartesianTarget(flange, motion: Motions.Joint);
                var publicSolution = system.Kinematics([cartesian], [joints])[0];

                Assert.Multiple(() =>
                {
                    Assert.That(matching.All(solution => solution.IsNearSingular),
                        Is.EqualTo(expected), message);
                    Assert.That(publicSolution.Errors.Contains("Target near singularity."),
                        Is.EqualTo(expected), message);
                });
            }
        }
    }

    [Test]
    public void PoWaShoulderBoundaryRoundTrips()
    {
        var robot = GetRobot(TestRobots.AbbPowa1920());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        const double joint2 = 2.5;
        double combined = Math.PI - Math.Asin(
            -(robot.Joints[0].A + robot.Joints[1].A * Math.Cos(joint2))
            / robot.Joints[3].D);

        double[] joints = [0.4, joint2, combined - joint2, -0.2, 0.5, 0.6];
        var solutions = AssertRoundTrip(robot, solver, joints, joints, sourceTolerance: 2e-5);
        Assert.That(solutions.Where(solution => SamePoseJoints(solution.Joints, joints, 2e-5))
            .All(solution => !solution.IsNearSingular), Is.True);
    }

    [TestCase(1e-2)]
    [TestCase(1e-4)]
    [TestCase(1e-6)]
    [TestCase(1e-7)]
    [TestCase(0)]
    public void Axis4BoundaryKeepsMultipleRoot(double offset)
    {
        var robot = GetRobot(TestRobots.AbbGofa12());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] joints = [0.2, 0.4, -0.5, Math.PI - offset, 0.7, -0.4];
        _ = AssertRoundTrip(robot, solver, joints, joints, sourceTolerance: 2e-5);
    }

    [Test]
    public void Axis4ZeroKeepsMultipleRoots()
    {
        foreach (var system in new[] { TestRobots.AbbGofa10(), TestRobots.AbbGofa12() })
        {
            var robot = GetRobot(system);
            var solver = (NonSphericalWristKinematics)robot.Solver;

            foreach (double joint4 in new[] { -1e-8, 1e-8 })
            {
                double[] joints = [-0.91, 0.83, -0.37, joint4, -1.03, 0.88];
                _ = AssertRoundTrip(robot, solver, joints, joints, $"{system.Name}, q4={joint4}", sourceTolerance: 2e-5);
            }
        }
    }

    [Test]
    public void Axis4HalfTurnRecoversBranch()
    {
        var robot = GetRobot(TestRobots.AbbGofa12());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] joints = [0.43, -0.39, 0.58, Math.PI - 1e-7, 0.86, -0.97];
        _ = AssertRoundTrip(robot, solver, joints, joints, "seeded", sourceTolerance: 2e-5);
        _ = AssertRoundTrip(robot, solver, joints, previous: null, "unseeded", sourceTolerance: 2e-5);
    }

    [Test]
    public void GoFaHalfTurnRecoversBranchWithoutPrevious()
    {
        var robot = GetRobot(TestRobots.AbbGofa10());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] joints =
        [
            2.9866559559602246,
            -1.5395721784005967,
            -1.4765879153515096,
            -3.141592634719776,
            Math.PI - 1e-3,
            3.141592653562408
        ];

        _ = AssertRoundTrip(robot, solver, joints, previous: null, sourceTolerance: 2e-5);
    }

    [Test]
    public void PreviousSelectsContinuousWinding()
    {
        var system = TestRobots.AbbPowa1920();
        double[] joints = [2 * Math.PI - 1e-4, 0.35, 0.55, -2 * Math.PI + 0.2, 0.4, 6.5];
        var flange = system.Kinematics([new JointTarget(joints)])[0].Planes[^1];
        var target = new CartesianTarget(flange, motion: Motions.Joint);
        var solution = system.Kinematics([target], [joints])[0];

        Assert.Multiple(() =>
        {
            Assert.That(solution.Errors, Is.Empty);
            Assert.That(solution.Joints, Is.EqualTo(joints).Within(2e-5));
        });
    }

    [Test]
    public void CompactSolveMatchesExhaustiveSelection()
    {
        foreach (var (factory, seed, _) in Cases)
            _ = AssertCompactSelection(factory(), seed, 4);
    }

    [Test]
    public void JointTargetSetsConfigurationAndChecksLimits()
    {
        var system = TestRobots.AbbGofa12();
        var robot = GetRobot(system);
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] joints = [-0.4, 0.5, -0.6, 0.7, -0.8, 0.9];
        var target = Forward(robot, joints);
        var candidates = solver.GetSolutions(target, joints, out var errors);
        var expectedConfiguration = candidates
            .Where(candidate => SamePoseJoints(candidate.Joints, joints))
            .Min(candidate => candidate.Configuration);

        var valid = system.Kinematics([new JointTarget(joints)])[0];
        double[] outOfRange = [.. joints];
        outOfRange[0] = robot.Joints[0].Range.T1 + 0.1;
        var invalid = system.Kinematics([new JointTarget(outOfRange)])[0];

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty);
            Assert.That(valid.Errors, Is.Empty);
            Assert.That(valid.Configuration, Is.EqualTo(expectedConfiguration));
            Assert.That(invalid.Configuration, Is.EqualTo(RobotConfigurations.Undefined));
            Assert.That(invalid.Errors, Has.Some.Contains("Axis 1 is outside the permitted range."));
        });
    }

    [Test]
    public void WristConfigurationIgnoresLegalWinding()
    {
        var system = TestRobots.AbbPowa1920();
        var robot = GetRobot(system);
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] wound = [0.3, 0.4, 0.5, -0.6, 3.5, 0.7];
        double[] principal = [.. wound];
        principal[4] = Normalize(principal[4]);
        var target = Forward(robot, wound);
        var solutions = solver.GetSolutions(target, wound, out var errors);
        var matching = solutions
            .Where(solution => SameLiftedJoints(solution.Joints, wound, 2e-5))
            .ToArray();

        var woundForward = system.Kinematics([new JointTarget(wound)])[0];
        var principalForward = system.Kinematics([new JointTarget(principal)])[0];

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty);
            Assert.That(matching, Is.Not.Empty);
            Assert.That(woundForward.Configuration, Is.EqualTo(principalForward.Configuration));
            Assert.That(matching.All(solution => solution.Configuration == woundForward.Configuration),
                Is.True);
        });
    }

    [TestCase(false, 0)]
    [TestCase(false, 1e-9)]
    [TestCase(true, 0)]
    [TestCase(true, 1e-9)]
    public void JointBoundariesRoundTrip(bool max, double inset)
    {
        var robot = GetRobot(TestRobots.AbbGofa12());
        var solver = (NonSphericalWristKinematics)robot.Solver;
        var range = robot.Joints[2].Range;
        double joint3 = max ? range.T1 - inset : range.T0 + inset;
        double[] joints = [-0.4, 0.5, joint3, 0.7, -0.8, 0.9];
        _ = AssertRoundTrip(robot, solver, joints, previous: null, sourceTolerance: 2e-5);
    }

    [Test]
    public void EnumeratesOnlyLegalWindings()
    {
        var system = TestRobots.AbbPowa1920();
        var robot = GetRobot(system);
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] joints = [0.3, 0.35, 0.55, 0.2, 0.4, 0.25];
        var rawTarget = Forward(robot, joints);
        var candidates = solver.GetSolutions(rawTarget, joints, out var errors);
        var equivalent = candidates.Where(solution => SamePoseJoints(solution.Joints, joints)).ToArray();
        var expectedWindings = EnumerateLegalWindings(robot, joints);

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty);
            Assert.That(equivalent, Has.Length.EqualTo(expectedWindings.Count));

            foreach (var expected in expectedWindings)
            {
                Assert.That(
                    equivalent.Any(solution => SameLiftedJoints(solution.Joints, expected)),
                    Is.True,
                    $"Missing winding [{string.Join(", ", expected)}].");
            }
        });

        double[] boundaryPrevious = [2 * Math.PI - 1e-4, joints[1], joints[2], joints[3], joints[4], joints[5]];
        double[] boundaryTargetJoints = [1e-4, joints[1], joints[2], joints[3], joints[4], joints[5]];
        var boundaryForward = system.Kinematics([new JointTarget(boundaryTargetJoints)])[0];
        var flange = boundaryForward.Planes[^1];
        var target = new CartesianTarget(flange, boundaryForward.Configuration, Motions.Joint);
        var selected = system.Kinematics([target], [boundaryPrevious])[0];

        Assert.Multiple(() =>
        {
            Assert.That(selected.Errors, Is.Empty);
            Assert.That(selected.Joints[0], Is.InRange(robot.Joints[0].Range.T0, robot.Joints[0].Range.T1));
            Assert.That(selected.Joints[0], Is.EqualTo(boundaryTargetJoints[0]).Within(2e-5));
        });
    }

    [Test]
    public void ForcedConfigurationFiltersOrFallsBack()
    {
        var system = TestRobots.AbbPowa1920();
        var robot = GetRobot(system);
        var solver = (NonSphericalWristKinematics)robot.Solver;
        double[] original =
        [
            2.576167123373895,
            4.040198554640063,
            1.7990458237853075,
            4.371644106806894,
            0.05724243904172033,
            -4.844716669238397
        ];

        double[] expectedJoints =
        [
            2.6279322928745463,
            4.06295303909917,
            1.8451558189406274,
            -2.4037566366766963,
            0.11461269708795258,
            1.9798894349364797
        ];

        var rawTarget = Forward(robot, original);
        var candidates = solver.GetSolutions(rawTarget, original, out var errors);
        var expectedConfiguration = RobotConfigurations.Shoulder | RobotConfigurations.Wrist;
        var expected = candidates.First(candidate =>
            candidate.Configuration == expectedConfiguration
            && SameLiftedJoints(candidate.Joints, expectedJoints, 2e-5));

        var branches = new List<WristSolution>();

        foreach (var candidate in candidates.Where(candidate =>
            candidate.Configuration == expectedConfiguration))
        {
            if (!branches.Any(alternative =>
                SamePoseJoints(alternative.Joints, candidate.Joints, 2e-5)))
            {
                branches.Add(candidate);
            }
        }

        var flange = system.Kinematics([new JointTarget(expected.Joints)])[0].Planes[^1];
        var target = new CartesianTarget(flange, expected.Configuration, Motions.Joint);
        var selected = system.Kinematics([target], [expected.Joints])[0];
        var available = candidates.Select(candidate => candidate.Configuration).ToHashSet();
        var unavailableConfiguration = Enumerable.Range(0, 8)
            .Select(value => (RobotConfigurations)value)
            .First(configuration => !available.Contains(configuration));
        var unavailableTarget = new CartesianTarget(
            flange,
            unavailableConfiguration,
            Motions.Joint);

        var unavailable = system.Kinematics([unavailableTarget], [expected.Joints])[0];

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty);
            Assert.That(branches, Has.Count.GreaterThan(1));
            Assert.That(selected.Errors, Is.Empty);
            Assert.That(selected.Configuration, Is.EqualTo(expected.Configuration));
            Assert.That(selected.Joints, Is.EqualTo(expected.Joints).Within(2e-5));
            Assert.That(unavailable.Errors, Does.Contain("Target configuration is not available."));
            Assert.That(unavailable.Configuration, Is.EqualTo(expected.Configuration));
            Assert.That(unavailable.Joints, Is.EqualTo(expected.Joints).Within(2e-5));
        });
    }

    [Test]
    public void UnreachableReturnsNoBranch()
    {
        foreach (var (factory, _, _) in Cases)
        {
            var robot = GetRobot(factory());
            var solver = (NonSphericalWristKinematics)robot.Solver;
            var target = Transform.Identity;
            target.M03 = 10_000;
            var solutions = solver.GetSolutions(target, previous: null, out var errors);

            Assert.Multiple(() =>
            {
                Assert.That(solutions, Is.Empty, robot.Model);
                Assert.That(errors, Does.Contain("Target out of reach."), robot.Model);
            });
        }
    }

    static int AssertCompactSelection(RobotSystem system, int seed, int count)
    {
        var robot = GetRobot(system);
        var solver = (NonSphericalWristKinematics)robot.Solver;
        var random = new Random(seed);
        int branches = 0;

        for (int sample = 0; sample < count; sample++)
        {
            var joints = RandomJoints(robot, random);
            var previous = new double[joints.Length];

            for (int i = 0; i < previous.Length; i++)
            {
                var range = robot.Joints[i].Range;
                double offset = (i & 1) == 0 ? 0.12 : -0.12;
                previous[i] = Math.Clamp(joints[i] + offset, range.T0, range.T1);
            }

            var rawTarget = Forward(robot, joints);
            var candidates = solver.GetSolutions(rawTarget, previous, out var errors);
            var flange = system.Kinematics([new JointTarget(joints)])[0].Planes[^1];
            string context = $"Seed {seed}, sample {sample}";

            Assert.That(errors, Is.Empty, context);
            Assert.That(candidates, Is.Not.Empty, context);
            AssertAllSolutions(robot, rawTarget, candidates);
            AssertConfigurations(robot, rawTarget, candidates);
            AssertSelection(configuration: null);

            foreach (var configuration in candidates
                .Select(candidate => candidate.Configuration)
                .Distinct())
            {
                AssertSelection(configuration);
            }

            branches += candidates.Count;

            void AssertSelection(RobotConfigurations? configuration)
            {
                var eligible = configuration is RobotConfigurations requested
                    ? candidates.Where(candidate => candidate.Configuration == requested)
                    : candidates;
                var expected = eligible
                    .OrderBy(candidate => SquaredDifference(candidate.Joints, previous))
                    .First();
                var target = new CartesianTarget(flange, configuration, Motions.Joint);
                var actual = system.Kinematics([target], [previous])[0];
                string message = $"{context}, configuration {configuration?.ToString() ?? "automatic"}";

                Assert.Multiple(() =>
                {
                    Assert.That(actual.Configuration, Is.EqualTo(expected.Configuration), message);
                    Assert.That(actual.Joints, Is.EqualTo(expected.Joints).Within(1e-6), message);
                    Assert.That(
                        actual.Errors.Contains("Target near singularity."),
                        Is.EqualTo(expected.IsNearSingular),
                        message);
                });
            }
        }

        return branches;
    }

    static List<WristSolution> AssertRoundTrip(
        RobotArm robot,
        NonSphericalWristKinematics solver,
        double[] joints,
        double[]? previous,
        string message = "",
        double sourceTolerance = 2e-6)
    {
        var target = Forward(robot, joints);
        var solutions = solver.GetSolutions(target, previous, out var errors);

        Assert.Multiple(() =>
        {
            Assert.That(errors, Is.Empty, message);
            Assert.That(solutions, Is.Not.Empty, message);
            Assert.That(
                solutions.Any(solution => SamePoseJoints(solution.Joints, joints, sourceTolerance)),
                Is.True,
                $"{message}: original branch was not recovered.");
        });

        AssertAllSolutions(robot, target, solutions);
        AssertConfigurations(robot, target, solutions);
        return solutions;
    }

    static double[] RandomJoints(RobotArm robot, Random random)
    {
        var joints = new double[6];

        for (int i = 0; i < joints.Length; i++)
        {
            var range = robot.Joints[i].Range;
            double margin = (range.T1 - range.T0) * 0.2;
            joints[i] = range.T0 + margin + random.NextDouble() * (range.T1 - range.T0 - 2 * margin);
        }

        if (Math.Abs(Math.Sin(joints[4])) < 0.2)
            joints[4] += 0.35;

        double bendBoundary = Math.Atan2(robot.Joints[3].D, robot.Joints[2].A);

        if (Math.Abs(Math.Sin(joints[2] - bendBoundary)) < 0.2)
            joints[2] += 0.35;

        return joints;
    }

    static double[] FullRangeJoints(RobotArm robot, Random random)
    {
        var joints = new double[6];

        for (int i = 0; i < joints.Length; i++)
        {
            var range = robot.Joints[i].Range;
            joints[i] = range.T0 + random.NextDouble() * (range.T1 - range.T0);
        }

        return joints;
    }

    static void AssertAllSolutions(
        RobotArm robot,
        Transform target,
        List<WristSolution> solutions)
    {
        foreach (var solution in solutions)
        {
            var actual = Forward(robot, solution.Joints);
            double dx = actual.M03 - target.M03;
            double dy = actual.M13 - target.M13;
            double dz = actual.M23 - target.M23;
            double positionError = Math.Sqrt(dx * dx + dy * dy + dz * dz);
            double orientationError = 0;

            for (int row = 0; row < 3; row++)
            {
                for (int column = 0; column < 3; column++)
                    orientationError = Math.Max(orientationError, Math.Abs(actual[row, column] - target[row, column]));
            }

            Assert.Multiple(() =>
            {
                Assert.That(positionError, Is.LessThan(1e-5));
                Assert.That(orientationError, Is.LessThan(1e-8));

                for (int i = 0; i < 6; i++)
                {
                    Assert.That(solution.Joints[i], Is.InRange(
                        robot.Joints[i].Range.T0 - 1e-10,
                        robot.Joints[i].Range.T1 + 1e-10));
                }
            });
        }

        for (int i = 0; i < solutions.Count - 1; i++)
        {
            for (int j = i + 1; j < solutions.Count; j++)
            {
                if (solutions[i].Configuration != solutions[j].Configuration)
                    continue;

                Assert.That(
                    SameLiftedJoints(solutions[i].Joints, solutions[j].Joints, 1e-7),
                    Is.False,
                    $"Duplicate solutions {i} and {j}.");
            }
        }
    }

    static void AssertConfigurations(
        RobotArm robot,
        Transform target,
        IReadOnlyList<WristSolution> solutions)
    {
        foreach (var solution in solutions)
        {
            double q6 = solution.Joints[5];
            double cos6 = Math.Cos(q6);
            double sin6 = Math.Sin(q6);
            var rx = new Vector3d(target.M00, target.M10, target.M20);
            var ry = new Vector3d(target.M01, target.M11, target.M21);
            var rz = new Vector3d(target.M02, target.M12, target.M22);
            var x5 = cos6 * rx - sin6 * ry;
            var y5 = sin6 * rx + cos6 * ry;
            var p5 = new Point3d(target.M03, target.M13, target.M23) - robot.Joints[5].D * rz;
            var p4 = p5 - robot.Joints[4].A * x5 - robot.Joints[4].D * y5;
            double radial = Math.Cos(solution.Joints[0]) * p4.X + Math.Sin(solution.Joints[0]) * p4.Y;
            double bend = Math.Sin(
                solution.Joints[2]
                - Math.Atan2(robot.Joints[3].D, robot.Joints[2].A));

            if (Math.Abs(radial) < 1e-6
                || Math.Abs(bend) < 1e-6
                || Math.Abs(Math.Sin(solution.Joints[4])) < 1e-6)
            {
                continue;
            }

            bool shoulder = radial < 0;
            bool elbow = bend > 0 ^ shoulder;
            bool wrist = Normalize(solution.Joints[4]) > 0;
            RobotConfigurations expected = RobotConfigurations.None;

            if (shoulder) expected |= RobotConfigurations.Shoulder;
            if (elbow) expected |= RobotConfigurations.Elbow;
            if (wrist) expected |= RobotConfigurations.Wrist;

            Assert.That(solution.Configuration, Is.EqualTo(expected));
        }
    }

    static Transform Forward(RobotArm robot, double[] joints)
    {
        var transform = Transform.Identity;

        for (int i = 0; i < 6; i++)
        {
            var jointDefinition = robot.Joints[i];
            double cos = Math.Cos(joints[i]);
            double sin = Math.Sin(joints[i]);
            double cosAlpha = Math.Cos(jointDefinition.Alpha);
            double sinAlpha = Math.Sin(jointDefinition.Alpha);
            Transform joint = default;
            joint.Set(
                cos, -sin * cosAlpha, sin * sinAlpha, jointDefinition.A * cos,
                sin, cos * cosAlpha, -cos * sinAlpha, jointDefinition.A * sin,
                0, sinAlpha, cosAlpha, jointDefinition.D);
            transform *= joint;
        }

        return transform;
    }

    static bool SamePoseJoints(
        double[] first,
        double[] second,
        double tolerance = 2e-6)
    {
        for (int i = 0; i < first.Length; i++)
        {
            if (Math.Abs(Math.IEEERemainder(first[i] - second[i], 2 * Math.PI)) > tolerance)
                return false;
        }

        return true;
    }

    static bool SameLiftedJoints(double[] first, double[] second, double tolerance = 1e-10)
    {
        for (int i = 0; i < first.Length; i++)
        {
            if (Math.Abs(first[i] - second[i]) > tolerance)
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

    static Plane PathPlane(double x, double y, double z) =>
        new(new(x, y, z), -Vector3d.XAxis, Vector3d.YAxis);

    static List<double[]> EnumerateLegalWindings(RobotArm robot, double[] principal)
    {
        var results = new List<double[]>();
        var current = new double[principal.Length];
        AddJoint(0);
        return results;

        void AddJoint(int index)
        {
            if (index == principal.Length)
            {
                results.Add([.. current]);
                return;
            }

            double angle = Normalize(principal[index]);
            var range = robot.Joints[index].Range;
            int minTurn = (int)Math.Ceiling((range.T0 - angle - 1e-10) / (2 * Math.PI));
            int maxTurn = (int)Math.Floor((range.T1 - angle + 1e-10) / (2 * Math.PI));

            for (int turn = minTurn; turn <= maxTurn; turn++)
            {
                current[index] = angle + turn * 2 * Math.PI;
                AddJoint(index + 1);
            }
        }
    }

    static double Normalize(double angle)
    {
        angle = Math.IEEERemainder(angle, 2 * Math.PI);
        return angle <= -Math.PI ? angle + 2 * Math.PI : angle;
    }

    static bool SupportsAfter(Action<Joint[]> change)
    {
        var robot = GetRobot(TestRobots.AbbGofa12());
        change(robot.Joints);
        return NonSphericalWristKinematics.Supports(robot);
    }

    static RobotArm GetRobot(RobotSystem system) =>
        ((IndustrialSystem)system).MechanicalGroups[0].Robot;
}
