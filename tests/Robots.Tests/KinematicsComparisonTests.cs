using System.Diagnostics;
using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

class KinematicsComparisonTests
{
    const int RoundTripCount = 16;
    const int BenchmarkCount = 32;
    const int WarmupCount = 512;
    const int MaxSolveAllocation = 16 * 1024;
    const double PositionTolerance = 1e-5;
    const double OrientationTolerance = 1e-8;

    static readonly (string Name, Func<RobotSystem> Factory, int Seed)[] Cases =
    [
        ("PoWa 1920", TestRobots.AbbPowa1920, 1920),
        ("GoFa 10", TestRobots.AbbGofa10, 15010),
        ("GoFa 12", TestRobots.AbbGofa12, 15012)
    ];

    [Test]
    public void SolverCapabilitiesAreDisjoint()
    {
        var spherical = GetRobot(TestRobots.AbbIrb120());

        Assert.Multiple(() =>
        {
            Assert.That(SphericalWristKinematics.Supports(spherical), Is.True);
            Assert.That(NonSphericalWristKinematics.Supports(spherical), Is.False);

            foreach (var (name, factory, _) in Cases)
            {
                var nonSpherical = GetRobot(factory());
                Assert.That(SphericalWristKinematics.Supports(nonSpherical), Is.False, name);
                Assert.That(NonSphericalWristKinematics.Supports(nonSpherical), Is.True, name);
            }
        });
    }

    [Test]
    public void SolversReconstructTargets()
    {
        foreach (var (name, factory, seed) in Cases)
        {
            var robot = GetRobot(factory());
            var analytical = robot.Solver;
            var numerical = new NumericalKinematics(robot);
            var samples = CreateSamples(robot, seed, RoundTripCount);

            for (int i = 0; i < samples.Length; i++)
            {
                var sample = samples[i];
                var analyticalSolution = analytical.Solve(sample.Target, new(sample.Previous), basePlane: null);
                var numericalSolution = numerical.Solve(sample.Target, new(sample.Previous), basePlane: null);
                var analyticalError = GetPoseError(sample.Expected, analyticalSolution.Planes[^1]);
                var numericalError = GetPoseError(sample.Expected, numericalSolution.Planes[^1]);

                Assert.Multiple(() =>
                {
                    Assert.That(analyticalSolution.Errors, Is.Empty, $"{name} analytical sample {i}");
                    AssertPose(analyticalError, $"{name} analytical sample {i}");
                    Assert.That(numericalSolution.Errors, Is.Empty, $"{name} numerical sample {i}");
                    AssertPose(numericalError, $"{name} numerical sample {i}");
                });
            }
        }
    }

    [Test]
    public void AllocationsStayBounded()
    {
        foreach (var (name, factory, seed) in Cases)
        {
            var robot = GetRobot(factory());
            var sample = CreateSamples(robot, seed, count: 1)[0];
            var previous = new PreviousJoints(sample.Previous);
            _ = robot.Solver.Solve(sample.Target, previous, basePlane: null);
            long allocationStart = GC.GetAllocatedBytesForCurrentThread();
            var solution = robot.Solver.Solve(sample.Target, previous, basePlane: null);
            long allocated = GC.GetAllocatedBytesForCurrentThread() - allocationStart;

            Assert.Multiple(() =>
            {
                Assert.That(solution.Errors, Is.Empty, name);
                Assert.That(allocated, Is.LessThan(MaxSolveAllocation), name);
            });
        }
    }

    [Test]
    [Explicit("Reports analytical, numerical, and spherical kinematics latency and allocations.")]
    public void BenchmarkSolvers()
    {
        foreach (var (name, factory, seed) in Cases)
        {
            var robot = GetRobot(factory());
            var samples = CreateSamples(robot, seed, BenchmarkCount);
            Report(name, "analytical warm", Measure(robot.Solver, samples, usePrevious: true));
            Report(name, "analytical forced", Measure(robot.Solver, samples, usePrevious: true, useConfiguredTarget: true));
            Report(name, "numerical warm", Measure(new NumericalKinematics(robot), samples, usePrevious: true));
            Report(name, "analytical cold", Measure(robot.Solver, samples, usePrevious: false));
            Report(name, "numerical cold", Measure(new NumericalKinematics(robot), samples, usePrevious: false));
        }

        var spherical = GetRobot(TestRobots.AbbIrb120());
        var sphericalSamples = CreateSamples(spherical, seed: 120, BenchmarkCount);
        var sphericalMeasurement = Measure(spherical.Solver, sphericalSamples, usePrevious: true);
        Report("IRB 120", "spherical warm", sphericalMeasurement);
        Assert.That(sphericalMeasurement.Successes, Is.EqualTo(sphericalMeasurement.Attempts));
    }

    static Sample[] CreateSamples(RobotArm robot, int seed, int count)
    {
        var random = new Random(seed);
        var samples = new Sample[count];

        for (int sampleIndex = 0; sampleIndex < samples.Length; sampleIndex++)
        {
            var joints = RandomJoints(robot, random);
            var previous = NearbyJoints(robot, joints);
            var forward = robot.Kinematics(new JointTarget(joints));
            var expected = forward.Planes[^1];
            var target = new CartesianTarget(expected, motion: Motions.Joint);
            var configuredTarget = new CartesianTarget(expected, forward.Configuration, Motions.Joint);
            samples[sampleIndex] = new(target, configuredTarget, previous, expected);
        }

        return samples;
    }

    static double[] RandomJoints(RobotArm robot, Random random)
    {
        var joints = new double[robot.Joints.Length];

        for (int i = 0; i < joints.Length; i++)
        {
            var range = robot.Joints[i].Range;
            double margin = (range.T1 - range.T0) * 0.2;
            joints[i] = range.T0 + margin + random.NextDouble() * (range.T1 - range.T0 - 2 * margin);
        }

        if (joints.Length == 6 && Math.Abs(Math.Sin(joints[4])) < 0.25)
            joints[4] = MoveInsideRange(robot.Joints[4].Range, joints[4], 0.4);

        return joints;
    }

    static double[] NearbyJoints(RobotArm robot, double[] joints)
    {
        var previous = new double[joints.Length];

        for (int i = 0; i < previous.Length; i++)
        {
            double offset = (i & 1) == 0 ? 0.06 : -0.06;
            previous[i] = MoveInsideRange(robot.Joints[i].Range, joints[i], offset);
        }

        return previous;
    }

    static double MoveInsideRange(Interval range, double value, double offset)
    {
        const double margin = 1e-4;
        return Math.Clamp(value + offset, range.T0 + margin, range.T1 - margin);
    }

    static Measurement Measure(
        MechanismKinematics solver,
        Sample[] samples,
        bool usePrevious,
        bool useConfiguredTarget = false)
    {
        for (int i = 0; i < WarmupCount; i++)
        {
            var sample = samples[i % samples.Length];
            var previous = usePrevious ? new PreviousJoints(sample.Previous) : default;
            var target = useConfiguredTarget ? sample.ConfiguredTarget : sample.Target;
            _ = solver.Solve(target, previous, basePlane: null);
        }

        GC.Collect();
        GC.WaitForPendingFinalizers();
        GC.Collect();

        var durations = new double[samples.Length];
        long allocated = 0;
        int successes = 0;
        double maxPositionError = 0;
        double maxOrientationError = 0;

        for (int i = 0; i < samples.Length; i++)
        {
            var sample = samples[i];
            var previous = usePrevious ? new PreviousJoints(sample.Previous) : default;
            var target = useConfiguredTarget ? sample.ConfiguredTarget : sample.Target;
            long allocationStart = GC.GetAllocatedBytesForCurrentThread();
            long timestamp = Stopwatch.GetTimestamp();
            var solution = solver.Solve(target, previous, basePlane: null);
            durations[i] = Stopwatch.GetElapsedTime(timestamp).TotalMilliseconds;
            allocated += GC.GetAllocatedBytesForCurrentThread() - allocationStart;
            var error = GetPoseError(sample.Expected, solution.Planes[^1]);
            maxPositionError = Math.Max(maxPositionError, error.Position);
            maxOrientationError = Math.Max(maxOrientationError, error.Orientation);

            if (solution.Errors.Count == 0
                && error.Position < PositionTolerance
                && error.Orientation < OrientationTolerance)
            {
                successes++;
            }
        }

        Array.Sort(durations);
        double total = 0;

        foreach (double duration in durations)
            total += duration;

        int p95Index = Math.Max(0, (int)Math.Ceiling(durations.Length * 0.95) - 1);
        var measurement = new Measurement(
            samples.Length,
            successes,
            total / durations.Length,
            Percentile(durations, 0.5),
            durations[p95Index],
            allocated / (double)samples.Length,
            maxPositionError,
            maxOrientationError);

        if (solver is NonSphericalWristKinematics)
            Assert.That(measurement.Successes, Is.EqualTo(measurement.Attempts));

        return measurement;
    }

    static double Percentile(double[] sorted, double percentile)
    {
        double index = (sorted.Length - 1) * percentile;
        int lower = (int)Math.Floor(index);
        int upper = (int)Math.Ceiling(index);
        double fraction = index - lower;
        return sorted[lower] + (sorted[upper] - sorted[lower]) * fraction;
    }

    static PoseError GetPoseError(Plane expected, Plane actual)
    {
        var expectedTransform = expected.ToTransform();
        var actualTransform = actual.ToTransform();
        double dx = expectedTransform.M03 - actualTransform.M03;
        double dy = expectedTransform.M13 - actualTransform.M13;
        double dz = expectedTransform.M23 - actualTransform.M23;
        double position = Math.Sqrt(dx * dx + dy * dy + dz * dz);
        double orientation = 0;

        for (int row = 0; row < 3; row++)
        {
            for (int column = 0; column < 3; column++)
            {
                orientation = Math.Max(
                    orientation,
                    Math.Abs(expectedTransform[row, column] - actualTransform[row, column]));
            }
        }

        return new(position, orientation);
    }

    static void AssertPose(PoseError error, string message)
    {
        Assert.Multiple(() =>
        {
            Assert.That(error.Position, Is.LessThan(PositionTolerance), $"{message} position");
            Assert.That(error.Orientation, Is.LessThan(OrientationTolerance), $"{message} orientation");
        });
    }

    static void Report(string robot, string solver, Measurement measurement)
    {
        TestContext.Out.WriteLine(
            $"{robot,-10} {solver,-18} "
            + $"success {measurement.Successes,2}/{measurement.Attempts,-2}  "
            + $"mean {measurement.MeanMs,8:0.###} ms  "
            + $"median {measurement.MedianMs,8:0.###} ms  "
            + $"p95 {measurement.P95Ms,8:0.###} ms  "
            + $"alloc {measurement.Bytes / 1024,9:0.###} KiB  "
            + $"max error {measurement.MaxPositionError:0.###e+0} mm / "
            + $"{measurement.MaxOrientationError:0.###e+0}");
    }

    static RobotArm GetRobot(RobotSystem system) =>
        ((IndustrialSystem)system).MechanicalGroups[0].Robot;

    readonly record struct Sample(
        CartesianTarget Target,
        CartesianTarget ConfiguredTarget,
        double[] Previous,
        Plane Expected);

    readonly record struct PoseError(double Position, double Orientation);

    readonly record struct Measurement(
        int Attempts,
        int Successes,
        double MeanMs,
        double MedianMs,
        double P95Ms,
        double Bytes,
        double MaxPositionError,
        double MaxOrientationError);
}
