using Rhino.Geometry;

namespace Robots.Tests;

static class TestRobots
{
    const string PostProcessorIOXml = """<IO><DO names="DO1"/><DI names="DI1"/><AO names="AO1"/><AI names="AI1"/></IO>""";

    static readonly string AbbIrb120ArmXml = AbbArmXml("IRB120", 3);
    static readonly string AbbCustomSlideXml = CustomExternalXml(Manufacturers.ABB, "Slide", x: 100, y: 20, movesRobot: true);

    static string AbbArmXml(string model, int payload) => $"""
        <RobotArm model="{model}" manufacturer="ABB" payload="{payload}">
          <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
          <Joints>
            <Revolute number="1" a="0" d="290" minrange="-165" maxrange="165" maxspeed="250"/>
            <Revolute number="2" a="270" d="0" minrange="-110" maxrange="110" maxspeed="250"/>
            <Revolute number="3" a="70" d="0" minrange="-110" maxrange="70" maxspeed="250"/>
            <Revolute number="4" a="0" d="302" minrange="-160" maxrange="160" maxspeed="320"/>
            <Revolute number="5" a="0" d="0" minrange="-120" maxrange="120" maxspeed="320"/>
            <Revolute number="6" a="0" d="72" minrange="-400" maxrange="400" maxspeed="420"/>
          </Joints>
        </RobotArm>
        """;

    public static readonly string AbbIrb120Xml = $"""
        <RobotSystem name="IRB120" manufacturer="ABB">
          <Mechanisms>
            {AbbIrb120ArmXml}
          </Mechanisms>
        </RobotSystem>
        """;

    public const string GripperToolXml = """
        <Tool name="Gripper">
          <Tcp x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
          <Mass weight="0" x="0" y="0" z="0"/>
        </Tool>
        """;

    static readonly string FanucLrMateXml = """
        <RobotSystem name="LRMate" manufacturer="Fanuc">
          <Mechanisms>
            <RobotArm model="LRMate" manufacturer="Fanuc" payload="7">
              <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
              <Joints>
                <Revolute number="1" a="0" d="330" minrange="-170" maxrange="170" maxspeed="350"/>
                <Revolute number="2" a="260" d="0" minrange="-100" maxrange="145" maxspeed="350"/>
                <Revolute number="3" a="75" d="0" minrange="-170" maxrange="170" maxspeed="400"/>
                <Revolute number="4" a="0" d="290" minrange="-190" maxrange="190" maxspeed="450"/>
                <Revolute number="5" a="0" d="0" minrange="-140" maxrange="140" maxspeed="450"/>
                <Revolute number="6" a="0" d="80" minrange="-360" maxrange="360" maxspeed="720"/>
              </Joints>
            </RobotArm>
          </Mechanisms>
        </RobotSystem>
        """;

    static readonly string AbbIrb120WithCustomExternalXml = $"""
        <RobotSystem name="IRB120External" manufacturer="ABB">
          <Mechanisms>
            {AbbCustomSlideXml}
            {AbbIrb120ArmXml}
          </Mechanisms>
        </RobotSystem>
        """;

    static readonly string AbbTwoGroupWithCustomExternalXml = $"""
        <RobotSystem name="IRB120ExternalFrame" manufacturer="ABB">
          <Mechanisms group="0">
            {AbbIrb120ArmXml}
          </Mechanisms>
          <Mechanisms group="1">
            {AbbCustomSlideXml}
            {AbbIrb120ArmXml}
          </Mechanisms>
        </RobotSystem>
        """;

    static readonly string AbbNumericalXml = $"""
        <RobotSystem name="CRB15000" manufacturer="ABB">
          <Mechanisms>
            {AbbArmXml("CRB15000Test", 5)}
          </Mechanisms>
        </RobotSystem>
        """;

    const string UR10Xml = """
        <RobotSystem name="UR10" manufacturer="UR">
          <Mechanisms>
            <RobotArm model="UR10" manufacturer="UR" payload="10">
              <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
              <Joints>
                <Revolute number="1" a="0" d="127.3" minrange="-360" maxrange="360" maxspeed="120"/>
                <Revolute number="2" a="-612" d="0" minrange="-360" maxrange="360" maxspeed="120"/>
                <Revolute number="3" a="-572.3" d="0" minrange="-360" maxrange="360" maxspeed="180"/>
                <Revolute number="4" a="0" d="163.941" minrange="-360" maxrange="360" maxspeed="180"/>
                <Revolute number="5" a="0" d="115.7" minrange="-360" maxrange="360" maxspeed="180"/>
                <Revolute number="6" a="0" d="92.2" minrange="-360" maxrange="360" maxspeed="180"/>
              </Joints>
            </RobotArm>
          </Mechanisms>
        </RobotSystem>
        """;

    public static RobotSystem AbbIrb120() => Parse(AbbIrb120Xml);

    public static RobotSystem AbbIrb120WithCustomExternal() => Parse(AbbIrb120WithCustomExternalXml);

    public static RobotSystem AbbTwoGroupWithCustomExternal() => Parse(AbbTwoGroupWithCustomExternalXml);

    public static RobotSystem AbbNumerical() => Parse(AbbNumericalXml);

    public static RobotSystem KukaWithCustomExternal() =>
        Parse(PostProcessorXml(Manufacturers.KUKA, 6, model: "KR", external: CustomExternalXml(Manufacturers.KUKA), io: ""));

    public static RobotSystem UR10() => Parse(UR10Xml);

    public static RobotSystem FanucLrMate() => Parse(FanucLrMateXml);

    public static RobotSystem PostProcessorRobot(Manufacturers manufacturer, int jointCount) =>
        manufacturer == Manufacturers.UR && jointCount == 6
            ? UR10()
            : Parse(PostProcessorXml(manufacturer, jointCount));

    public static Program AbbSampleProgram()
    {
        var planeA = Plane.WorldYZ;
        var planeB = Plane.WorldYZ;
        planeA.Origin = new(300, 200, 610);
        planeB.Origin = new(300, -200, 610);

        return SampleProgram("TestProgram", AbbIrb120(), planeA, planeB);
    }

    public static Program URSampleProgram()
    {
        var planeA = Plane.WorldZX;
        var planeB = Plane.WorldZX;
        planeA.Origin = new(200, 100, 600);
        planeB.Origin = new(700, 250, 600);

        return SampleProgram("URTest", UR10(), planeA, planeB);
    }

    public static SimpleToolpath Toolpath(params Target[] targets)
    {
        return new(targets);
    }

    public static string FlattenCode(Program program)
    {
        var code = program.Code ?? throw new InvalidOperationException("Program code was not generated.");
        return string.Join("\n", code.SelectMany(group => group).SelectMany(file => file)).ReplaceLineEndings("\n");
    }

    static RobotSystem Parse(string xml) => FileIO.ParseRobotSystem(xml, Plane.WorldXY);

    static Program SampleProgram(string name, RobotSystem robot, Plane planeA, Plane planeB)
    {
        var speed = new Speed(300);
        var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
        var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);

        return new(name, robot, [Toolpath(targetA, targetB)]);
    }

    static string PostProcessorXml(Manufacturers manufacturer, int jointCount, string? model = null, string? external = null, string io = PostProcessorIOXml)
    {
        var joints = (manufacturer, jointCount) switch
        {
            (Manufacturers.FrankaEmika, 7) => """
              <Revolute number="1" a="0" d="333" minrange="-360" maxrange="360" maxspeed="180"/>
              <Revolute number="2" a="0" d="0" minrange="-360" maxrange="360" maxspeed="180"/>
              <Revolute number="3" a="82.5" d="316" minrange="-360" maxrange="360" maxspeed="180"/>
              <Revolute number="4" a="-82.5" d="0" minrange="-360" maxrange="360" maxspeed="180"/>
              <Revolute number="5" a="0" d="384" minrange="-360" maxrange="360" maxspeed="180"/>
              <Revolute number="6" a="88" d="0" minrange="-360" maxrange="360" maxspeed="180"/>
              <Revolute number="7" a="0" d="107" minrange="-360" maxrange="360" maxspeed="180"/>
              """,
            _ => string.Concat(Enumerable.Range(1, jointCount).Select(i =>
                $"""<Revolute number="{i}" a="0" d="0" minrange="-360" maxrange="360" maxspeed="180"/>"""))
        };

        return $"""
            <RobotSystem name="{manufacturer}" manufacturer="{manufacturer}">
              <Mechanisms>
                <RobotArm model="{model ?? manufacturer.ToString()}" manufacturer="{manufacturer}" payload="10">
                  <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
                  <Joints>{joints}</Joints>
                </RobotArm>
                {external}
              </Mechanisms>
              {io}
            </RobotSystem>
            """;
    }

    static string CustomExternalXml(Manufacturers manufacturer, string model = "External", int x = 0, int y = 0, bool movesRobot = false) => $"""
            <Custom model="{model}" manufacturer="{manufacturer}" payload="0"{(movesRobot ? " movesRobot=\"true\"" : "")}>
              <Base x="{x}" y="{y}" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints><Prismatic number="7" a="0" d="0" minrange="-1000" maxrange="1000" maxspeed="1000"/></Joints>
            </Custom>
            """;
}
