using Rhino.Geometry;

namespace Robots;

public class Frame : TargetAttribute
{
    public static Frame Default { get; } = new Frame(Plane.WorldXY, -1, -1, "DefaultFrame");

    /// <summary>
    /// Reference frame of plane for a target
    /// </summary>

    Plane _plane;
    public ref Plane Plane => ref _plane;
    public int CoupledMechanism { get; }
    public int CoupledMechanicalGroup { get; }
    /// <summary>
    /// Specifies that it will use a tool that exists in the controller and does not need to be defined in the generated program.
    /// </summary>
    public bool UseController { get; }
    /// <summary>
    /// Used only in KUKA to load from the BASE_DATA array.
    /// </summary>
    public int? Number { get; }
    internal int CoupledPlaneIndex { get; set; }
    public bool IsCoupled => (CoupledMechanicalGroup != -1);

    public Frame(Plane plane, int coupledMechanism = -1, int coupledMechanicalGroup = -1, string? name = null, bool useController = false, int? number = null)
        : base(name)
    {
        Plane = plane;
        CoupledMechanism = coupledMechanism;
        CoupledMechanicalGroup = coupledMechanicalGroup;
        UseController = number is not null || useController;
        Number = number;

        if (number is not null && number.Value < 1)
            throw new ArgumentOutOfRangeException(nameof(number), " Frame number out of range.");
    }

    public Frame ShallowClone() => (Frame)MemberwiseClone();

    public override string ToString()
    {
        if (HasName)
            return $"Frame ({Name})";

        string origin = $"{Plane.OriginX:0.##},{Plane.OriginY:0.##},{Plane.OriginZ:0.##}";
        return $"Frame ({origin}" + (IsCoupled ? " Coupled" : "") + ")";
    }
}
