using NUnit.Framework;
namespace Robots.Tests;

public class TargetTests
{
    [TestCase(5)]
    [TestCase(8)]
    public void JointTargetsRejectUnsupportedJointCounts(int jointCount)
    {
        var joints = new double[jointCount];

        var exception = Assert.Throws<ArgumentOutOfRangeException>(() =>
        {
            _ = new JointTarget(joints);
        });

        Assert.That(exception, Is.Not.Null);
    }

    [Test]
    public void JointTargetsRejectNonFiniteValues()
    {
        double[] joints = [0, 0, double.NaN, 0, 0, 0];

        var exception = Assert.Throws<ArgumentException>(() => new JointTarget(joints));

        Assert.That(exception!.Message, Does.Contain("Joint value 2 must be finite."));
    }
}
