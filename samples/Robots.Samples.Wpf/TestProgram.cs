using Rhino.Geometry;
using Plane = Rhino.Geometry.Plane;

namespace Robots.Samples.Wpf;

class TestProgram
{
    public static async Task<Program> Create()
    {
        var robot = await GetRobot();

        var planeA = Plane.WorldYZ;
        var planeB = Plane.WorldYZ;
        planeA.Origin = new(300, 200, 610);
        planeB.Origin = new(300, -200, 610);
        Speed speed = new(300);
        CartesianTarget targetA = new(planeA, RobotConfigurations.Wrist, Motions.Joint);
        CartesianTarget targetB = new(planeB, null, Motions.Linear, speed: speed);
        SimpleToolpath toolpath = new(targetA, targetB);

        return new("TestProgram", robot, [toolpath]);
    }

    static async Task<RobotSystem> GetRobot()
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

            await DownloadLibrary();
            return FileIO.LoadRobotSystem(name, Plane.WorldXY);
        }
    }

    static async Task DownloadLibrary()
    {
        OnlineLibrary online = new();
        await online.UpdateLibrary();
        var bartlett = online.Libraries["Bartlett"];
        await online.DownloadLibrary(bartlett);
    }
}
