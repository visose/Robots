using System.Diagnostics;
using Rhino.Geometry;
using Robots;

var mesh = new Mesh();
var times = new Dictionary<string, long>();
int count = 20;
var watch = new Stopwatch();

for (int i = 0; i < count; i++)
    PerfTest();

foreach (var time in times)
    Console.WriteLine($"{time.Key}: {time.Value / count} ms");

//dotnet run --property:Configuration=Release

void PerfTest()
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
    Debug.Assert(program.Duration == expected, "PROGRAM ERROR");
}

void Log(string key)
{
    watch.Stop();
    long ms = watch.ElapsedMilliseconds;
    times[key] = times.ContainsKey(key)
        ? times[key] + ms
        : ms;

    watch.Restart();
}