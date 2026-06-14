using System.Diagnostics;

using NUnit.Framework;

using Rhino.Geometry;

namespace Robots.Tests;

class Performance
{
    const int Iterations = 20;
    const double DurationTolerance = 1e-8;

    [Test]
    [Explicit("Reports synthetic ABB parse and program creation timings.")]
    public void AbbIrb120ProgramCreation() =>
        Report("ABB IRB120", TestRobots.AbbIrb120, AbbProgram, 3.7851345985264309);

    [Test]
    [Explicit("Reports synthetic UR parse and program creation timings.")]
    public void UR10ProgramCreation() =>
        Report("UR10", TestRobots.UR10, URProgram, 1.7425724724517486);

    static Program AbbProgram(RobotSystem robot)
    {
        var planeA = Plane.WorldYZ;
        var planeB = Plane.WorldYZ;
        planeA.Origin = new(300, 200, 610);
        planeB.Origin = new(300, -200, 610);
        Speed speed = new(300);
        JointTarget targetA = new([0, Math.PI * 0.5, 0, 0, 0, 0]);
        CartesianTarget targetB = new(planeA, RobotConfigurations.Wrist, Motions.Joint);
        CartesianTarget targetC = new(planeB, null, Motions.Linear, speed: speed);
        SimpleToolpath toolpath = [targetA, targetB, targetC];

        return new("TestProgram", robot, [toolpath], stepSize: 0.02);
    }

    static Program URProgram(RobotSystem robot)
    {
        var planeA = Plane.WorldZX;
        var planeB = Plane.WorldZX;
        planeA.Origin = new(200, 100, 600);
        planeB.Origin = new(700, 250, 600);
        Speed speed = new(300);
        CartesianTarget targetA = new(planeA, RobotConfigurations.Wrist, Motions.Joint);
        CartesianTarget targetB = new(planeB, null, Motions.Linear, speed: speed);
        SimpleToolpath toolpath = [targetA, targetB];

        return new("URTest", robot, [toolpath], stepSize: 0.01);
    }

    static void Report(string label, Func<RobotSystem> robotFactory, Func<RobotSystem, Program> programFactory, double expectedDuration)
    {
        long parseMilliseconds = 0;
        long programMilliseconds = 0;
        Program? program = null;

        for (int i = 0; i < Iterations; i++)
        {
            var (robot, parseElapsed) = Measure(robotFactory);
            parseMilliseconds += parseElapsed;

            (program, long programElapsed) = Measure(() => programFactory(robot));
            programMilliseconds += programElapsed;
        }

        TestContext.Out.WriteLine($"{label} parse average: {Average(parseMilliseconds):0.###} ms");
        TestContext.Out.WriteLine($"{label} program average: {Average(programMilliseconds):0.###} ms");

        Assert.That(program, Is.Not.Null);
        Assert.That(program!.Errors, Is.Empty);
        Assert.That(program.Duration, Is.EqualTo(expectedDuration).Within(DurationTolerance));
    }

    static (T Result, long ElapsedMilliseconds) Measure<T>(Func<T> action)
    {
        var watch = Stopwatch.StartNew();
        var result = action();
        watch.Stop();
        return (result, watch.ElapsedMilliseconds);
    }

    static double Average(long totalMilliseconds) => totalMilliseconds / (double)Iterations;
}
