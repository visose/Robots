using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class ABBTests
{
    readonly Program _program;

    public ABBTests()
    {
        const string xml = "<RobotSystem name=\"IRB120\" manufacturer=\"ABB\"><Mechanisms><RobotArm model=\"IRB120\" manufacturer=\"ABB\" payload=\"3\"><Base x=\"0.000\" y=\"0.000\" z=\"0.000\" q1=\"1.000\" q2=\"0.000\" q3=\"0.000\" q4=\"0.000\"/><Joints><Revolute number=\"1\" a =\"0\" d =\"290\" minrange = \"-165\" maxrange =\"165\" maxspeed =\"250\"/><Revolute number=\"2\" a =\"270\" d =\"0\" minrange = \"-110\" maxrange =\"110\" maxspeed =\"250\"/><Revolute number=\"3\" a =\"70\" d =\"0\" minrange = \"-110\" maxrange =\"70\" maxspeed =\"250\"/><Revolute number=\"4\" a =\"0\" d =\"302\" minrange = \"-160\" maxrange =\"160\" maxspeed =\"320\"/><Revolute number=\"5\" a =\"0\" d =\"0\" minrange = \"-120\" maxrange =\"120\" maxspeed =\"320\"/><Revolute number=\"6\" a =\"0\" d =\"72\" minrange = \"-400\" maxrange =\"400\" maxspeed =\"420\"/></Joints></RobotArm></Mechanisms><IO><DO names=\"DO10_1,DO10_2\"/><DI names=\"DI10_1,DI10_2\"/></IO></RobotSystem>";
        var robot = FileIO.ParseRobotSystem(xml, Plane.WorldXY);

        var planeA = Plane.WorldYZ;
        var planeB = Plane.WorldYZ;
        planeA.Origin = new Point3d(300, 200, 610);
        planeB.Origin = new Point3d(300, -200, 610);
        var speed = new Speed(300);
        var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
        var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
        var toolpath = new SimpleToolpath() { targetA, targetB };

        _program = new Program("TestProgram", robot, new[] { toolpath });
    }

    [Test]
    public void AbbCorrectDuration()
    {
        const double expected = 1.6432545251573487;
        Assert.That(_program.Duration, Is.EqualTo(expected).Within(1e-14));
    }

    [Test]
    public void AbbCorrectJoints()
    {
        double[] expected =
        [
            -0.72007069377409672,
            1.5806369662963811,
            -0.075569321979534809,
            -1.4960590345094886,
            0.72252891341688164,
            3.042104596978858
        ];

        var actual = _program.Targets[1].Joints;

        Assert.That(actual, Is.EqualTo(expected).Within(1e-14));
    }

    [Test]
    public void AbbCorrectPlanes()
    {
        double[] expected =
        [
            0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -0, 290, -0.7517591128712748, 0.6594378183081357, 1.2246467991473532E-16, -9.206393913076605E-17, 8.075784134277723E-17, -1, -1.9973711765339164, 1.752079979415716, 559.9869269504153, 0.007397671024199599, -0.006489185108947015, -0.9999515812978345, 0.7517227136706884, -0.6594058891848145, 0.009840480677307488, 1.4589883621218538, -1.2798143527384684, 629.8357722916104, -0.04937656483793966, 0.043312776173631114, -0.9978406477313588, 0.6594378183081357, 0.7517591128712748, -1.2220023553033302E-16, 228.00000000000006, -200.00000000000006, 609.9999999999999, -0.6612838130259876, -0.7464264865721835, -0.07450650155064824, 0.7501358001254245, -0.6580138597591445, -0.06568136520400843, 228.00000000000006, -200.00000000000006, 609.9999999999999, 2.7755575615628914E-16, -0.9950551439451086, -0.09932401778210068, 2.4980018054066027E-16, 0.09932401778210068, -0.9950551439451086, 300.00000000000006, -200.00000000000006, 609.9999999999999, -2.5137212532089737E-16, 1, 3.2162452993532737E-16, -2.761329074653924E-16, -3.2162452993532737E-16, 1, 300.00000000000006, -200.00000000000006, 609.9999999999999, -2.5137212532089737E-16, 1, 3.2162452993532737E-16, -2.761329074653924E-16, -3.216245299353274E-16, 1
        ];

        var actual = _program.Targets[1].Planes
            .SelectMany(p => new[] { (Vector3d)p.Origin, p.XAxis, p.YAxis })
            .SelectMany(v => new[] { v.X, v.Y, v.Z });

        Assert.That(actual, Is.EqualTo(expected).Within(1e-12));
    }

    [Test]
    public void AbbCorrectCode()
    {
        const string expected = @"MODULE TestProgram_T_ROB1
VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];
VAR confdata conf := [0,0,0,0];
PERS tooldata DefaultTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
TASK PERS wobjdata DefaultFrame:=[FALSE,TRUE,"""",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
TASK PERS speeddata DefaultSpeed:=[100,180,5000,1080];
TASK PERS speeddata Speed000:=[300,180,5000,1080];
PROC Main()
ConfL \Off;
MoveAbsJ [[41.257,-0.5638,4.3298,85.7179,-41.3979,5.7002],extj],DefaultSpeed,fine,DefaultTool;
MoveL [[300,-200,610],[0.5,0.5,0.5,0.5],conf,extj],Speed000,fine,DefaultTool \WObj:=DefaultFrame;
ENDPROC
ENDMODULE";

        var code = _program.Code ?? throw new InvalidOperationException("Program code not generated");
        var actual = string.Join(Environment.NewLine, code[0].SelectMany(c => c));

        Assert.That(actual, Is.EqualTo(expected));
    }
}
