using NUnit.Framework;
using Rhino.Geometry;

namespace Robots.Tests;

public class TargetTests
{
    [TestCase(true)]
    [TestCase(false)]
    public void AttributeHashesDistributeNonTranslationValues(bool speed)
    {
        const int count = 1000;
        HashSet<int> hashes = [];
        HashSet<TargetProperty> attributes = [];

        for (int i = 0; i < count; i++)
        {
            TargetProperty first = speed ? new Speed(time: i) : new Zone(10, rotation: i);
            TargetProperty equal = speed ? new Speed(time: i) : new Zone(10, rotation: i);
            Assert.That(first.GetHashCode(), Is.EqualTo(equal.GetHashCode()));
            _ = hashes.Add(first.GetHashCode());
            _ = attributes.Add(first);
            _ = attributes.Add(equal);
        }

        Assert.Multiple(() =>
        {
            Assert.That(attributes, Has.Count.EqualTo(count));
            Assert.That(hashes.Count, Is.GreaterThan(count * 0.95), "Attributes must not all share their translation-only hash.");
        });
    }

    [Test]
    public void CartesianTargetsRejectUndefinedInputConfiguration()
    {
        var exception = Assert.Throws<ArgumentOutOfRangeException>(() =>
            _ = new CartesianTarget(Plane.WorldXY, RobotConfigurations.Undefined));

        Assert.That(exception!.ParamName, Is.EqualTo("configuration"));
    }

    [TestCase(5)]
    [TestCase(8)]
    public void JointTargetsAllowMechanismSpecificJointCounts(int jointCount)
    {
        var joints = new double[jointCount];
        var target = new JointTarget(joints);

        Assert.That(target.Joints, Is.SameAs(joints));
    }

    [Test]
    public void JointTargetsRejectNonFiniteValues()
    {
        double[] joints = [0, 0, double.NaN, 0, 0, 0];

        var exception = Assert.Throws<ArgumentException>(() => new JointTarget(joints));

        Assert.That(exception!.Message, Does.Contain("Joint value 2 must be finite."));
    }
}
