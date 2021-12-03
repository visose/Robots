using Rhino.Geometry;

namespace Robots;

public class Frame : TargetAttribute
{
    public static Frame Default { get; } = new Frame(Plane.WorldXY, -1, -1, "DefaultFrame");

    /// <summary>
    /// Reference frame of plane for a target
    /// </summary>
    public Plane Plane { get; internal set; }
    public int CoupledMechanism { get; }
    public int CoupledMechanicalGroup { get; }
    internal int CoupledPlaneIndex { get; set; }
    public bool IsCoupled => (CoupledMechanicalGroup != -1);

    public Frame(Plane plane, int coupledMechanism = -1, int coupledMechanicalGroup = -1, string? name = null)
    {
        Name = name;
        Plane = plane;
        CoupledMechanism = coupledMechanism;
        CoupledMechanicalGroup = coupledMechanicalGroup;
    }

    public Frame ShallowClone() => (Frame)MemberwiseClone();

    public override string ToString()
    {
        if (Name is not null)
            return $"Frame ({Name})";

        string origin = $"{Plane.OriginX:0.##},{Plane.OriginY:0.##},{Plane.OriginZ:0.##}";
        return $"Frame ({origin}" + (IsCoupled ? " Coupled" : "") + ")";
    }
}
