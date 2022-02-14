using Rhino.Geometry;
using Plane = Rhino.Geometry.Plane;

namespace Robots.Samples.Wpf;

class TestProgram
{
    public static Program Create()
    {
        var robot = FileIO.LoadRobotSystem("Bartlett-IRB120", Plane.WorldXY);

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
}