using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class ManufacturerTests
{
    static IEnumerable<Manufacturers> Supported =>
        Enum.GetValues<Manufacturers>().Where(manufacturer => manufacturer != Manufacturers.All);

    [TestCaseSource(nameof(Supported))]
    public void JointAnglesRoundTrip(Manufacturers manufacturer)
    {
        int jointCount = manufacturer == Manufacturers.FrankaEmika ? 7 : 6;
        var system = (IndustrialSystem)TestRobots.PostProcessorRobot(manufacturer, jointCount);
        var robot = system.MechanicalGroups[0].Robot;

        Assert.Multiple(() =>
        {
            Assert.That(ManufacturerCatalog.Get(manufacturer).Id, Is.EqualTo(manufacturer));

            for (int joint = 0; joint < robot.Joints.Length; joint++)
            {
                double radians = robot.DegreeToRadian(30, joint);
                Assert.That(robot.RadianToDegree(radians, joint), Is.EqualTo(30).Within(1e-12), $"Axis {joint + 1}");
            }
        });
    }

    [Test]
    public void KukaControllerIOUsesOneBasedIndices()
    {
        var io = new IO(Manufacturers.KUKA, true, [], [], [], []);

        Assert.Multiple(() =>
        {
            Assert.That(io.ValidateBounds(0, io.DO), Is.EqualTo("IO index is out of range."));
            Assert.That(io.ValidateBounds(1, io.DO), Is.Null);
        });
    }

    [TestCase(Manufacturers.ABB)]
    [TestCase(Manufacturers.All)]
    [TestCase(Manufacturers.UR)]
    public void UnsupportedControllerIONumberingFails(Manufacturers manufacturer)
    {
        var exception = Assert.Throws<NotSupportedException>(
            () => _ = new IO(manufacturer, true, [], [], [], []));

        Assert.That(
            exception!.Message,
            Is.EqualTo($"Controller IO numbering is not supported for {manufacturer} robots."));
    }

    [Test]
    public void RobotArmManufacturerMustMatchSystem()
    {
        string xml = TestRobots.AbbIrb120Xml.Replace(
            "<RobotArm model=\"IRB120\" manufacturer=\"ABB\"",
            "<RobotArm model=\"IRB120\" manufacturer=\"KUKA\"",
            StringComparison.Ordinal);

        var exception = Assert.Throws<ArgumentException>(
            () => FileIO.ParseRobotSystem(xml, Plane.WorldXY));

        Assert.That(
            exception!.Message,
            Is.EqualTo("Robot system manufacturer ABB does not match robot arm manufacturer KUKA."));
    }
}
