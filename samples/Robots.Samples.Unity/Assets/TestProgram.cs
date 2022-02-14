using Rhino.Geometry;
using System;
using System.Threading.Tasks;

namespace Robots.Samples.Unity
{
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
            var cellName = "Bartlett-IRB120";

            try
            {
                return FileIO.LoadRobotSystem(cellName, Plane.WorldXY);
            }
            catch (ArgumentException e)
            {
                if (!e.Message.Contains("not found"))
                    throw;

                UnityEngine.Debug.Log("Bartlett robot library not found, installing...");
                await DownloadLibraryAsync();
                return FileIO.LoadRobotSystem(cellName, Plane.WorldXY);
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
}