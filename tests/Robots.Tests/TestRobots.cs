using Rhino.Geometry;

namespace Robots.Tests;

static class TestRobots
{
    static readonly string AbbIrb120ArmXml = AbbArmXml("IRB120", 3);

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

    const string CustomSlideXml = """
        <Custom model="Slide" manufacturer="ABB" payload="0" movesRobot="true">
          <Base x="100" y="20" z="0" q1="1" q2="0" q3="0" q4="0"/>
          <Joints>
            <Prismatic number="7" a="0" d="0" minrange="-1000" maxrange="1000" maxspeed="1000"/>
          </Joints>
        </Custom>
        """;

    const string AbbIOXml = """
        <IO>
          <DO names="DO10_1,DO10_2"/>
          <DI names="DI10_1,DI10_2"/>
        </IO>
        """;

    static readonly string AbbIrb120Xml = $"""
        <RobotSystem name="IRB120" manufacturer="ABB">
          <Mechanisms>
            {AbbIrb120ArmXml}
          </Mechanisms>
          {AbbIOXml}
        </RobotSystem>
        """;

    static readonly string AbbIrb120WithCustomExternalXml = $"""
        <RobotSystem name="IRB120External" manufacturer="ABB">
          <Mechanisms>
            {CustomSlideXml}
            {AbbIrb120ArmXml}
          </Mechanisms>
          {AbbIOXml}
        </RobotSystem>
        """;

    static readonly string AbbTwoGroupWithCustomExternalXml = $"""
        <RobotSystem name="IRB120ExternalFrame" manufacturer="ABB">
          <Mechanisms group="0">
            {AbbIrb120ArmXml}
          </Mechanisms>
          <Mechanisms group="1">
            {CustomSlideXml}
            {AbbIrb120ArmXml}
          </Mechanisms>
          {AbbIOXml}
        </RobotSystem>
        """;

    static readonly string AbbNumericalXml = $"""
        <RobotSystem name="CRB15000" manufacturer="ABB">
          <Mechanisms>
            {AbbArmXml("CRB15000Test", 5)}
          </Mechanisms>
          <IO>
            <DO names="DO10_1"/>
            <DI names="DI10_1"/>
          </IO>
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
          <IO>
            <DO names="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16"/>
            <DI names="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16"/>
            <AO names="0,1"/>
            <AI names="0,1"/>
          </IO>
        </RobotSystem>
        """;

    public static RobotSystem AbbIrb120() => Parse(AbbIrb120Xml);

    public static RobotSystem AbbIrb120WithCustomExternal() => Parse(AbbIrb120WithCustomExternalXml);

    public static RobotSystem AbbTwoGroupWithCustomExternal() => Parse(AbbTwoGroupWithCustomExternalXml);

    public static RobotSystem AbbNumerical() => Parse(AbbNumericalXml);

    public static RobotSystem UR10() => Parse(UR10Xml);

    public static RobotSystem PostProcessorRobot(Manufacturers manufacturer, int jointCount) =>
        manufacturer == Manufacturers.UR && jointCount == 6
            ? UR10()
            : Parse(PostProcessorXml(manufacturer, jointCount));

    static RobotSystem Parse(string xml) => FileIO.ParseRobotSystem(xml, Plane.WorldXY);

    static string PostProcessorXml(Manufacturers manufacturer, int jointCount)
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
                <RobotArm model="{manufacturer}" manufacturer="{manufacturer}" payload="10">
                  <Base x="0" y="0" z="0" q1="1" q2="0" q3="0" q4="0"/>
                  <Joints>{joints}</Joints>
                </RobotArm>
              </Mechanisms>
              <IO>
                <DO names="DO1"/>
                <DI names="DI1"/>
                <AO names="AO1"/>
                <AI names="AI1"/>
              </IO>
            </RobotSystem>
            """;
    }
}
