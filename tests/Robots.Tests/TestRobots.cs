using Rhino.Geometry;

namespace Robots.Tests;

static class TestRobots
{
    const string PostProcessorIOXml = """<IO><DO names="DO1"/><DI names="DI1"/><AO names="AO1"/><AI names="AI1"/></IO>""";

    static readonly string AbbIrb120ArmXml = AbbArmXml("IRB120", 3);
    static readonly string AbbCustomSlideXml = CustomExternalXml(Manufacturers.ABB, "Slide", x: 100, y: 20, movesRobot: true);

    static string AbbArmXml(string model, int payload, int wristOffset = 0, int axis4Offset = 0) => $"""
        <RobotArm model="{model}" manufacturer="ABB" payload="{payload}">
          <Base x="0.000" y="0.000" z="0.000" q1="1.000" q2="0.000" q3="0.000" q4="0.000"/>
          <Joints>
            <Revolute number="1" a="0" d="290" minrange="-165" maxrange="165" maxspeed="250"/>
            <Revolute number="2" a="270" d="0" minrange="-110" maxrange="110" maxspeed="250"/>
            <Revolute number="3" a="70" d="0" minrange="-110" maxrange="70" maxspeed="250"/>
            <Revolute number="4" a="{axis4Offset}" d="302" minrange="-160" maxrange="160" maxspeed="320"/>
            <Revolute number="5" a="{wristOffset}" d="0" minrange="-120" maxrange="120" maxspeed="320"/>
            <Revolute number="6" a="0" d="72" minrange="-400" maxrange="400" maxspeed="420"/>
          </Joints>
        </RobotArm>
        """;

    static string AbbIrb120SystemXml(bool omniCore = false) => $"""
        <RobotSystem name="IRB120" manufacturer="ABB"{(omniCore ? " controller=\"omnicore\"" : "")}>
          <Mechanisms>
            {AbbIrb120ArmXml}
          </Mechanisms>
          {PostProcessorIOXml}
        </RobotSystem>
        """;

    public static readonly string AbbIrb120Xml = AbbIrb120SystemXml();

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

    static readonly string AbbThreeGroupXml = $"""
        <RobotSystem name="IRB120ThreeGroup" manufacturer="ABB">
          <Mechanisms group="0">
            {AbbIrb120ArmXml}
          </Mechanisms>
          <Mechanisms group="1">
            {AbbIrb120ArmXml}
          </Mechanisms>
          <Mechanisms group="2">
            {AbbIrb120ArmXml}
          </Mechanisms>
        </RobotSystem>
        """;

    static readonly string AbbNumericalXml = $"""
        <RobotSystem name="OffsetWristTest" manufacturer="ABB">
          <Mechanisms>
            {AbbArmXml("OffsetWristTest", 5, wristOffset: 80, axis4Offset: 10)}
          </Mechanisms>
        </RobotSystem>
        """;

    const string AbbPowa1920Xml = """
        <RobotSystem name="PoWa1920Test" manufacturer="ABB">
          <Mechanisms>
            <RobotArm model="PoWa1920Test" manufacturer="ABB" payload="16">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints>
                <Revolute number="1" a="180" d="382.5" minrange="-360" maxrange="360" maxspeed="180"/>
                <Revolute number="2" a="920" d="0" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="3" a="0" d="-60.1" minrange="-248" maxrange="80" maxspeed="180"/>
                <Revolute number="4" a="0" d="856" minrange="-360" maxrange="360" maxspeed="180"/>
                <Revolute number="5" a="80" d="138" minrange="-270" maxrange="270" maxspeed="180"/>
                <Revolute number="6" a="0" d="120" minrange="-400" maxrange="400" maxspeed="180"/>
              </Joints>
            </RobotArm>
          </Mechanisms>
        </RobotSystem>
        """;

    const string AbbGofa10Xml = """
        <RobotSystem name="GoFa10Test" manufacturer="ABB">
          <Mechanisms>
            <RobotArm model="GoFa10Test" manufacturer="ABB" payload="10">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints>
                <Revolute number="1" a="150" d="399" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="2" a="707" d="0" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="3" a="110" d="0" minrange="-225" maxrange="85" maxspeed="180"/>
                <Revolute number="4" a="0" d="636" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="5" a="80" d="0" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="6" a="0" d="100.5" minrange="-180" maxrange="180" maxspeed="180"/>
              </Joints>
            </RobotArm>
          </Mechanisms>
        </RobotSystem>
        """;

    const string AbbGofa12Xml = """
        <RobotSystem name="GoFa12Test" manufacturer="ABB">
          <Mechanisms>
            <RobotArm model="GoFa12Test" manufacturer="ABB" payload="12">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints>
                <Revolute number="1" a="0" d="338" minrange="-270" maxrange="270" maxspeed="180"/>
                <Revolute number="2" a="707" d="0" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="3" a="110" d="0" minrange="-225" maxrange="85" maxspeed="180"/>
                <Revolute number="4" a="0" d="534" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="5" a="80" d="0" minrange="-180" maxrange="180" maxspeed="180"/>
                <Revolute number="6" a="0" d="101" minrange="-270" maxrange="270" maxspeed="180"/>
              </Joints>
            </RobotArm>
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

    static readonly string Ur10WithCustomExternalXml = UR10Xml.Replace(
        "</RobotArm>",
        $"</RobotArm>{CustomExternalXml(Manufacturers.UR)}",
        StringComparison.Ordinal);

    const string FrankaPandaXml = """
        <RobotSystem name="PandaTest" manufacturer="FrankaEmika">
          <Mechanisms>
            <RobotArm model="PandaTest" manufacturer="FrankaEmika" payload="3">
              <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints>
                <Revolute number="1" a="0" d="333" minrange="-166" maxrange="166" maxspeed="150"/>
                <Revolute number="2" a="0" d="0" minrange="-101" maxrange="101" maxspeed="150"/>
                <Revolute number="3" a="0" d="316" minrange="-166" maxrange="166" maxspeed="150"/>
                <Revolute number="4" a="82.5" d="0" minrange="-176" maxrange="-4" maxspeed="150"/>
                <Revolute number="5" a="-82.5" d="384" minrange="-166" maxrange="166" maxspeed="180"/>
                <Revolute number="6" a="0" d="0" minrange="-1" maxrange="215" maxspeed="180"/>
                <Revolute number="7" a="88" d="107" minrange="-166" maxrange="166" maxspeed="180"/>
              </Joints>
            </RobotArm>
          </Mechanisms>
        </RobotSystem>
        """;

    static readonly string FrankaPandaWithCustomExternalXml = FrankaPandaXml.Replace(
        "</RobotArm>",
        $"</RobotArm>{CustomExternalXml(Manufacturers.FrankaEmika, jointNumber: 8)}",
        StringComparison.Ordinal);

    public static RobotSystem AbbIrb120(bool omniCore = false) => Parse(AbbIrb120SystemXml(omniCore));

    public static RobotSystem AbbIrb120WithCustomExternal() => Parse(AbbIrb120WithCustomExternalXml);

    public static RobotSystem AbbTwoGroupWithCustomExternal() => Parse(AbbTwoGroupWithCustomExternalXml);

    public static RobotSystem AbbThreeGroup() => Parse(AbbThreeGroupXml);

    public static RobotSystem AbbNumerical() => Parse(AbbNumericalXml);

    public static RobotSystem AbbPowa1920() => Parse(AbbPowa1920Xml);

    public static RobotSystem AbbGofa10() => Parse(AbbGofa10Xml);

    public static RobotSystem AbbGofa12() => Parse(AbbGofa12Xml);

    public static RobotSystem KukaWithCustomExternal() =>
        Parse(PostProcessorXml(Manufacturers.KUKA, 6, model: "KR", external: CustomExternalXml(Manufacturers.KUKA), io: ""));

    public static RobotSystem KukaTwoGroupWithCustomExternal() =>
        Parse(AbbTwoGroupWithCustomExternalXml.Replace("manufacturer=\"ABB\"", "manufacturer=\"KUKA\"", StringComparison.Ordinal));

    public static RobotSystem UR10() => Parse(UR10Xml);

    public static RobotSystem UR10WithCustomExternal() => Parse(Ur10WithCustomExternalXml);

    public static RobotSystem DoosanWithCustomExternal() =>
        Parse(PostProcessorXml(Manufacturers.Doosan, 6, external: CustomExternalXml(Manufacturers.Doosan)));

    public static RobotSystem FanucLrMate() => Parse(FanucLrMateXml);

    public static RobotSystem SphericalRobot(Manufacturers manufacturer) =>
        Parse(AbbIrb120Xml.Replace("manufacturer=\"ABB\"", $"manufacturer=\"{manufacturer}\"", StringComparison.Ordinal));

    public static RobotSystem FrankaPanda(string? postProcessor = null) =>
        ParseFranka(FrankaPandaXml, postProcessor);

    public static RobotSystem FrankaPandaWithCustomExternal(string? postProcessor = null) =>
        ParseFranka(FrankaPandaWithCustomExternalXml, postProcessor);

    static RobotSystem ParseFranka(string xml, string? postProcessor)
    {
        if (postProcessor is not null)
        {
            xml = xml.Replace(
                "<RobotSystem ",
                $"<RobotSystem postProcessor=\"{postProcessor}\" ",
                StringComparison.Ordinal);
        }

        return Parse(xml);
    }

    public static RobotSystem PostProcessorRobot(
        Manufacturers manufacturer,
        int jointCount,
        string? postProcessor = null,
        IPostProcessor? postProcessorOverride = null,
        string? io = null)
    {
        if (postProcessor is not null || postProcessorOverride is not null || io is not null)
        {
            var xml = PostProcessorXml(
                manufacturer,
                jointCount,
                io: io ?? PostProcessorIOXml,
                postProcessor: postProcessor);
            return FileIO.ParseRobotSystem(xml, Plane.WorldXY, postProcessorOverride);
        }

        return (manufacturer, jointCount) switch
        {
            (Manufacturers.ABB, 6) => AbbIrb120(),
            (Manufacturers.UR, 6) => UR10(),
            _ => Parse(PostProcessorXml(manufacturer, jointCount))
        };
    }

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
        return string.Join("\n", code.SelectMany(group => group).SelectMany(file => file)).UseLF();
    }

    static RobotSystem Parse(string xml) => FileIO.ParseRobotSystem(xml, Plane.WorldXY);

    static Program SampleProgram(string name, RobotSystem robot, Plane planeA, Plane planeB)
    {
        var speed = new Speed(300);
        var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
        var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);

        return new(name, robot, [Toolpath(targetA, targetB)]);
    }

    static string PostProcessorXml(Manufacturers manufacturer, int jointCount, string? model = null, string? external = null, string io = PostProcessorIOXml, string? postProcessor = null)
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
            <RobotSystem name="{manufacturer}" manufacturer="{manufacturer}"{(postProcessor is null ? "" : $" postProcessor=\"{postProcessor}\"")}>
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

    static string CustomExternalXml(Manufacturers manufacturer, string model = "External", int x = 0, int y = 0, bool movesRobot = false, int jointNumber = 7) => $"""
            <Custom model="{model}" manufacturer="{manufacturer}" payload="0"{(movesRobot ? " movesRobot=\"true\"" : "")}>
              <Base x="{x}" y="{y}" z="0" q1="1" q2="0" q3="0" q4="0"/>
              <Joints><Prismatic number="{jointNumber}" a="0" d="0" minrange="-1000" maxrange="1000" maxspeed="1000"/></Joints>
            </Custom>
            """;
}
