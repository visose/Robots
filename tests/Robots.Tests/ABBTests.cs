using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class ABBTests
{
    readonly Program _program;

    public ABBTests()
    {
        const string xml = "<RobotCell name=\"IRB120\" manufacturer=\"ABB\"><Mechanisms><RobotArm model=\"IRB120\" manufacturer=\"ABB\" payload=\"3\"><Base x=\"0.000\" y=\"0.000\" z=\"0.000\" q1=\"1.000\" q2=\"0.000\" q3=\"0.000\" q4=\"0.000\"/><Joints><Revolute number=\"1\" a =\"0\" d =\"290\" minrange = \"-165\" maxrange =\"165\" maxspeed =\"250\"/><Revolute number=\"2\" a =\"270\" d =\"0\" minrange = \"-110\" maxrange =\"110\" maxspeed =\"250\"/><Revolute number=\"3\" a =\"70\" d =\"0\" minrange = \"-110\" maxrange =\"70\" maxspeed =\"250\"/><Revolute number=\"4\" a =\"0\" d =\"302\" minrange = \"-160\" maxrange =\"160\" maxspeed =\"320\"/><Revolute number=\"5\" a =\"0\" d =\"0\" minrange = \"-120\" maxrange =\"120\" maxspeed =\"320\"/><Revolute number=\"6\" a =\"0\" d =\"72\" minrange = \"-400\" maxrange =\"400\" maxspeed =\"420\"/></Joints></RobotArm></Mechanisms><IO><DO names=\"DO10_1,DO10_2\"/><DI names=\"DI10_1,DI10_2\"/></IO></RobotCell>";
        var robot = RobotSystem.Parse(xml, Plane.WorldXY);

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
        Assert.AreEqual(expected, _program.Duration, 1e-14);
    }

    [Test]
    public void AbbCorrectJoints()
    {
        double[] expected =
        {
                -0.72007069377409672,
                1.5806369662963811,
                -0.075569321979534809,
                -1.4960590345094886,
                0.72252891341688164,
                3.042104596978858
            };

        var actual = _program.Targets[1].Joints;

        for (int i = 0; i < 6; i++)
            Assert.AreEqual(expected[i], actual[i], 1e-14);
    }

    [Test]
    public void AbbCorrectPlanes()
    {
        double[] expected =
        {
            0, 0, 0, 1, 0, 0, 0, 1, 0, 0.7517591128712748, -0.6594378183081357, 290, -0.751759112871275, 0.6594378183081356, 0, 0.6594378183081356, 0.751759112871275, -0, -2.7564915612288043, 2.4179750537094775, 560.9770380510358, 0.5367791717546959, -0.4708589225918386, -0.7001142733768505, 0.6594378183081359, 0.7517591128712747, -4.6875121084248966E-17, 1.458988362121854, -1.2798143527384682, 629.8357722916104, -0.04937656483793966, 0.04331277617363111, -0.9978406477313588, 0.6594378183081357, 0.7517591128712748, -1.2220023553033302E-16, 228.66128381302607, -199.25357351342788, 610.0745065015506, -0.46759826847957253, -0.4575705438040238, -0.7562942924270918, 0.46759826847957286, 0.5980359168208571, -0.6509261874491932, 229.00000000000006, -200.00000000000003, 609.9999999999999, 3.925231146709438E-16, -0.6333775534297259, -0.7738429264465592, -7.850462293418875E-17, 0.7738429264465592, -0.6333775534297259, 300.00000000000006, -200.00000000000006, 609.9999999999999, -2.775557561562891E-16, 1, 3.1468563603142004E-16, -2.775557561562892E-16, -3.1468563603142014E-16, 1, 300.00000000000006, -200.00000000000006, 609.9999999999999, -2.775557561562891E-16, 1, 3.1468563603142004E-16, -2.775557561562892E-16, -3.1468563603142014E-16, 1
        };

        var actual = _program.Targets[1].Planes
            .SelectMany(p => new[] { (Vector3d)p.Origin, p.XAxis, p.YAxis })
            .SelectMany(v => new[] { v.X, v.Y, v.Z });

        foreach(var (e, a) in expected.Zip(actual))
            Assert.AreEqual(e, a, 1e-14);
    }

    [Test]
    public void AbbCorrectCode()
    {
        const string expected  = @"MODULE TestProgram_T_ROB1
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

        var code = _program.Code ?? throw new ArgumentNullException("Code not generated.");
        var actual = string.Join(Environment.NewLine, code[0].SelectMany(c => c));
        Assert.AreEqual(expected, actual);
    }
}