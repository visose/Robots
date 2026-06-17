using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class AbbRemoteIntegrationTests
{
    [Test]
    [Explicit("Requires ROBOTS_ABB_CONTROLLER_IP, ABB runtime, and a RobotStudio virtual controller.")]
    public void UploadToRobotStudioVirtualController()
    {
        var remote = CreateRemote();
        remote.Upload(CreateProgram());

        Assert.That(remote.Log.Any(log =>
            log.Contains("uploaded", StringComparison.OrdinalIgnoreCase)
            || log.Contains("loaded", StringComparison.OrdinalIgnoreCase)), Is.True);
    }

    [Test]
    [Explicit("Requires ROBOTS_ABB_CONTROLLER_IP, ABB runtime, and a RobotStudio virtual controller.")]
    public void PlayPauseRobotStudioVirtualController()
    {
        var remote = CreateRemote();
        remote.Play();
        remote.Pause();

        Assert.That(remote.Log, Is.Not.Empty);
    }

    static RemoteAbb CreateRemote()
    {
        string ip = Environment.GetEnvironmentVariable("ROBOTS_ABB_CONTROLLER_IP")
            ?? throw new IgnoreException("ROBOTS_ABB_CONTROLLER_IP is not set.");

        var helperPath = Environment.GetEnvironmentVariable("ROBOTS_ABB_REMOTE_HELPER");

        if (!string.IsNullOrWhiteSpace(helperPath) && !File.Exists(helperPath))
            throw new IgnoreException($"ROBOTS_ABB_REMOTE_HELPER was not found: {helperPath}");

        var remote = (RemoteAbb)(CreateProgram().RobotSystem.Remote
            ?? throw new InvalidOperationException("ABB remote was not created."));

        remote.IP = ip;
        return remote;
    }

    static Program CreateProgram()
    {
        var robot = TestRobots.AbbIrb120();
        var planeA = Plane.WorldYZ;
        var planeB = Plane.WorldYZ;
        planeA.Origin = new(300, 200, 610);
        planeB.Origin = new(300, -200, 610);
        var toolpath = new SimpleToolpath(
            new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint),
            new CartesianTarget(planeB, null, Motions.Linear, speed: new(300)));

        return new("TestProgram", robot, [toolpath]);
    }
}
