using System.Diagnostics;
using Rhino.Geometry;
using Robots;

//var mesh = new Mesh();
Dictionary<string, long> times = new();
Stopwatch watch = new();
int count = 20;

for (int i = 0; i < count; i++)
    PerfTestAbb();
    //PerfTestUR();

foreach (var time in times)
    Console.WriteLine($"{time.Key}: {time.Value / (count-1)} ms");

//dotnet run --property:Configuration=Release

void Log(string key)
{
    watch.Stop();
    long ms = watch.ElapsedMilliseconds;
    times[key] = times.ContainsKey(key)
        ? times[key] + ms
        : 0;

    watch.Restart();
}

void PerfTestAbb()
{
    watch.Restart();
    const string xml = "<RobotCell name=\"IRB120\" manufacturer=\"ABB\"><Mechanisms><RobotArm model=\"IRB120\" manufacturer=\"ABB\" payload=\"3\"><Base x=\"0.000\" y=\"0.000\" z=\"0.000\" q1=\"1.000\" q2=\"0.000\" q3=\"0.000\" q4=\"0.000\"/><Joints><Revolute number=\"1\" a =\"0\" d =\"290\" minrange = \"-165\" maxrange =\"165\" maxspeed =\"250\"/><Revolute number=\"2\" a =\"270\" d =\"0\" minrange = \"-110\" maxrange =\"110\" maxspeed =\"250\"/><Revolute number=\"3\" a =\"70\" d =\"0\" minrange = \"-110\" maxrange =\"70\" maxspeed =\"250\"/><Revolute number=\"4\" a =\"0\" d =\"302\" minrange = \"-160\" maxrange =\"160\" maxspeed =\"320\"/><Revolute number=\"5\" a =\"0\" d =\"0\" minrange = \"-120\" maxrange =\"120\" maxspeed =\"320\"/><Revolute number=\"6\" a =\"0\" d =\"72\" minrange = \"-400\" maxrange =\"400\" maxspeed =\"420\"/></Joints></RobotArm></Mechanisms><IO><DO names=\"DO10_1,DO10_2\"/><DI names=\"DI10_1,DI10_2\"/></IO></RobotCell>";
    var robot = RobotSystem.Parse(xml, Plane.WorldXY);

    Log("RobotSytem.Parse");

    var planeA = Plane.WorldYZ;
    var planeB = Plane.WorldYZ;
    planeA.Origin = new Point3d(300, 200, 610);
    planeB.Origin = new Point3d(300, -200, 610);
    var speed = new Speed(300);
    var targetA = new JointTarget(new[] { 0, Math.PI * 0.5, 0, 0, 0, 0 });
    var targetB = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
    var targetC = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
    var toolpath = new SimpleToolpath() { targetA, targetB, targetC };

    Log("Toolpath");

    var program = new Robots.Program("TestProgram", robot, new[] { toolpath }, stepSize: 0.02);

    Log("Program"); // 486

    double expected = 3.7851345985264309;
    Debug.Assert(program.Duration == expected, "Test failed");
}

void PerfTestUR()
{
    watch.Restart();
    const string xml = "<RobotCell name=\"UR10\" manufacturer=\"UR\"><Mechanisms><RobotArm model=\"UR10\" manufacturer=\"UR\" payload=\"10\"><Base x=\"0.000\" y=\"0.000\" z=\"0.000\" q1=\"1.000\" q2=\"0.000\" q3=\"0.000\" q4=\"0.000\"/><Joints><Revolute number=\"1\" a =\"0\" d =\"127.3\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"120\"/><Revolute number=\"2\" a =\"-612\" d =\"0\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"120\"/><Revolute number=\"3\" a =\"-572.3\" d =\"0\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/><Revolute number=\"4\" a =\"0\" d =\"163.941\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/><Revolute number=\"5\" a =\"0\" d =\"115.7\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/><Revolute number=\"6\" a =\"0\" d =\"92.2\" minrange = \"-360\" maxrange =\"360\" maxspeed =\"180\"/></Joints></RobotArm></Mechanisms><IO><DO names=\"0,1\"/><DI names=\"0,1\"/><AO names=\"0,1\"/><AI names=\"0,1\"/></IO></RobotCell>";
    var robot = RobotSystem.Parse(xml, Plane.WorldXY);

    Log("RobotSytem.Parse");

    var planeA = Plane.WorldZX;
    var planeB = Plane.WorldZX;
    planeA.Origin = new Point3d(200, 100, 600);
    planeB.Origin = new Point3d(700, 250, 600);
    var speed = new Speed(300);
    var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
    var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
    var toolpath = new SimpleToolpath() { targetA, targetB };

    Log("Toolpath");

    var program = new Robots.Program("URTest", robot, new[] { toolpath }, stepSize: 0.01);

    Log("Program");

    double expected = 1.7425724724517486;
    var err = Math.Abs(program.Duration - expected);
    Debug.Assert(err < 1e-9, "Test failed");
}