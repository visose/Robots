using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class URTests
{
    readonly Program _program;

    public URTests()
    {
        const string xml = "<RobotCell name=\"UR10\" manufacturer=\"UR\"><Mechanisms><RobotArm model=\"UR10\" manufacturer=\"UR\" payload=\"10\"><Base x=\"0.000\" y=\"0.000\" z=\"0.000\" q1=\"1.000\" q2=\"0.000\" q3=\"0.000\" q4=\"0.000\"/><Joints><Revolute number=\"1\" a =\"0\" d =\"127.3\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"120\"/><Revolute number=\"2\" a =\"-612\" d =\"0\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"120\"/><Revolute number=\"3\" a =\"-572.3\" d =\"0\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/><Revolute number=\"4\" a =\"0\" d =\"163.941\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/><Revolute number=\"5\" a =\"0\" d =\"115.7\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/><Revolute number=\"6\" a =\"0\" d =\"92.2\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/></Joints></RobotArm></Mechanisms><IO><DO names=\"0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16\"/><DI names=\"0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16\"/><AO names=\"0,1\"/><AI names=\"0,1\"/></IO></RobotCell>";
        var robot = FileIO.ParseRobotSystem(xml, Plane.WorldXY);

        var planeA = Plane.WorldZX;
        var planeB = Plane.WorldZX;
        planeA.Origin = new Point3d(200, 100, 600);
        planeB.Origin = new Point3d(700, 250, 600);
        var speed = new Speed(300);
        var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
        var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
        var toolpath = new SimpleToolpath() { targetA, targetB };

        _program = new Program("URTest", robot, new[] { toolpath });
    }

    [Test]
    public void URCorrectDuration()
    {
        const double expected = 1.7425663263380393;
        Assert.AreEqual(expected, _program.Duration, 1e-14);
    }

    [Test]
    public void URCorrectJoints()
    {
        double[] expected =
        {
            3.132810991378919, -1.2818634714414483, 1.6947375202478607, 2.728718604783381, 0.008781662210846086, -3.141592653589793
        };

        var actual = _program.Targets[1].Joints;

        for (int i = 0; i < 6; i++)
            Assert.AreEqual(expected[i], actual[i], 1e-14);
    }

    [Test]
    public void URCorrectPlanes()
    {
        double[] expected =
        {
            0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 127.3, 0.9999614414522032, -0.008781549341203346, 1.2246467991473532E-16, 1.2245995785452142E-16, -1.0754296292259227E-18, -1, 174.3701166078775, -1.5312988272824801, 713.931671117454, 0.2849184911893422, -0.0025021222668014376, 0.9585484822180622, 0.9585115219805949, -0.00841754079253349, -0.28492947765622506, 698.5603440194541, -6.134678673115655, 484.29999999999995, 0.9159360954247361, -0.008043648166753755, -0.4012435280752298, -0.4012280567074744, 0.003523539839631141, -0.9159714139522817, 700.0000000000003, 157.8, 484.29999999999995, -0.9999614414522032, 0.00878154934120347, -1.2246467991473532E-16, -0.00878154934120347, -0.9999614414522032, -1.4997597826618576E-32, 700.0000000000003, 157.8, 600, -1, 2.809905086387232E-14, -2.4492463776925676E-16, -2.4492463776925676E-16, 1.0754296292293638E-18, 1, 700.000000000003, 250, 600, -1.2245995785452142E-16, 1.0754296292259227E-18, 1, 1, -2.809905086387232E-14, 1.2245995785452144E-16, 700.000000000003, 250, 600, -1.2245995785452142E-16, 1.0754296292259227E-18, 1, 1, -2.809905086387232E-14, 1.2245995785452144E-16
        };

        var actual = _program.Targets[1].Planes
            .SelectMany(p => new[] { (Vector3d)p.Origin, p.XAxis, p.YAxis })
            .SelectMany(v => new[] { v.X, v.Y, v.Z });

        foreach (var (e, a) in expected.Zip(actual))
            Assert.AreEqual(e, a, 1e-14);
    }

    [Test]
    public void URCorrectCode()
    {
        const string expected = @"def Program():
  DefaultToolTcp = p[0, 0, 0, 0, 0, 1.5708]
  DefaultToolWeight = 0
  DefaultToolCog = [0, 0, 0]
  DefaultSpeed = 0.1
  Speed000 = 0.3
  DefaultZone = 0
  set_tcp(DefaultToolTcp)
  set_payload(DefaultToolWeight, DefaultToolCog)
  movej([2.2208, -2.4093, 2.5006, 3.0503, 0.9208, -3.1416], a=3.1416, v=0.2094, r=DefaultZone)
  movel(p[0.7, 0.25, 0.6, -1.2092, -1.2092, -1.2092], a=1, v=Speed000, r=DefaultZone)
end";

        var code = _program.Code ?? throw new InvalidOperationException("Program code not generated");
        var actual = string.Join(Environment.NewLine, code[0].SelectMany(c => c)).ReplaceLineEndings();
        Assert.AreEqual(expected, actual);
    }
}