using Rhino.Geometry;
using Plane = Rhino.Geometry.Plane;

namespace Robots.Samples.Wpf;

class TestProgram
{
    public static async Task<Program> CreateAsync()
    {
        var robot = await GetRobotAsync();

        var planeA = Plane.WorldYZ;
        var planeB = Plane.WorldYZ;
        planeA.Origin = new Point3d(300, 200, 610);
        planeB.Origin = new Point3d(300, -200, 610);
        var speed = new Speed(300);
        var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
        var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
        var toolpath = new SimpleToolpath() { targetA, targetB };

        return new Program("TestProgram", robot, new[] { toolpath });
    }

    static async Task<RobotSystem> GetRobotAsync()
    {
        var name = "Bartlett-IRB120";

        try
        {
            return FileIO.LoadRobotSystem(name, Plane.WorldXY);
        }
        catch (ArgumentException e)
        {
            if (!e.Message.Contains("not found"))
                throw;

            await DownloadLibraryAsync();
            return FileIO.LoadRobotSystem(name, Plane.WorldXY);
        }
    }

    static async Task DownloadLibraryAsync()
    {
        var online = new OnlineLibrary();
        await online.UpdateLibraryAsync();
        var bartlett = online.Libraries["Bartlett"];
        await online.DownloadLibraryAsync(bartlett);
    }
}
