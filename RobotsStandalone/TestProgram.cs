using Robots;
using Rhino.Geometry;
using Plane = Rhino.Geometry.Plane;

namespace RobotsStandalone
{
    class TestProgram
    {
        public Program Program { get; set; }

        public TestProgram()
        {
            var robot = RobotSystem.Load("Bartlett-IRB120", Plane.WorldXY);
            
            var planeA = Plane.WorldYZ;
            var planeB = Plane.WorldYZ;
            planeA.Origin = new Point3d(300, 200, 610);
            planeB.Origin = new Point3d(300, -200, 610);
            var speed = new Speed(300);
            var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
            var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
            var toolpath = new SimpleToolpath() { targetA, targetB };

            Program = new Program("TestProgram", robot, new[] { toolpath });
        }
    }
}
