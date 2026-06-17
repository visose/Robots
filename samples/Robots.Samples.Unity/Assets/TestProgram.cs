using System;
using System.Threading.Tasks;
using Rhino.Geometry;

namespace Robots.Samples.Unity
{
    class TestProgram
    {
        public static async Task<Program> CreateAsync()
        {
            var robot = await GetRobotAsync();

            var planeA = Plane.WorldYZ;
            var planeB = Plane.WorldYZ;
            planeA.Origin = new(300, 200, 610);
            planeB.Origin = new(300, -200, 610);
            Speed speed = new(300);
            CartesianTarget targetA = new(planeA, RobotConfigurations.Wrist, Motions.Joint);
            CartesianTarget targetB = new(planeB, null, Motions.Linear, speed: speed);
            SimpleToolpath toolpath = new(targetA, targetB);

            return new("TestProgram", robot, new[] { toolpath });
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

                UnityEngine.Debug.Log("Bartlett robot library not found, installing...");
                await DownloadLibraryAsync();
                return FileIO.LoadRobotSystem(name, Plane.WorldXY);
            }
        }

        static async Task DownloadLibraryAsync()
        {
            OnlineLibrary online = new();
            await online.UpdateLibraryAsync();
            var bartlett = online.Libraries["Bartlett"];
            await online.DownloadLibraryAsync(bartlett);
        }
    }
}
