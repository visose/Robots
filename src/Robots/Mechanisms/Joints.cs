using Rhino.Geometry;

namespace Robots;

public abstract class Joint
{
    public int Index { get; set; }
    public int Number { get; set; }
    internal double A { get; set; }
    internal double D { get; set; }
    public Interval Range { get; internal set; }
    public double MaxSpeed { get; internal set; }
    public Plane Plane { get; set; }
    public Mesh? Mesh { get; set; }
}

public class BaseJoint
{
}

public class RevoluteJoint : Joint
{
}

public class PrismaticJoint : Joint
{
}
