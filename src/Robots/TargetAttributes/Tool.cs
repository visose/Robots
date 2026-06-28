using Rhino.Geometry;

namespace Robots;

public class Tool : TargetProperty
{
    public static Tool Default { get; } = new(Plane.WorldXY, "DefaultTool");

    public Plane Tcp { get; }
    public double Weight { get; }
    public Point3d Centroid { get; }
    public Mesh Mesh { get; }
    public Mesh CollisionMesh { get; }

    /// <summary>
    /// Specifies that it will use a tool that exists in the controller and does not need to be defined in the generated program.
    /// </summary>
    public bool UseController { get; }

    /// <summary>
    /// Used only in KUKA to load from the TOOL_DATA array.
    /// </summary>
    public int? Number { get; }

    public Tool(Plane tcp, string name = "DefaultTool", double weight = 0, Point3d? centroid = null, Mesh? mesh = null, IReadOnlyList<Plane>? calibrationPlanes = null, bool useController = false, int? number = null, Mesh? collisionMesh = null)
        : base(name)
    {
        if (!tcp.IsValid)
            throw new ArgumentException("TCP plane is invalid.", nameof(tcp));

        Weight = CheckNonNegative(weight, nameof(weight));
        Centroid = (centroid is null) ? tcp.Origin : (Point3d)centroid;
        Mesh = mesh ?? FileIO.EmptyMesh;
        CollisionMesh = collisionMesh ?? Mesh;
        UseController = number is not null || useController;
        Number = number;

        if (number is not null)
            ArgumentOutOfRangeException.ThrowIfLessThan(number.Value, 1, nameof(number));

        if (calibrationPlanes is null || calibrationPlanes.Count == 0)
        {
            Tcp = tcp;
        }
        else
        {
            ArgumentOutOfRangeException.ThrowIfNotEqual(calibrationPlanes.Count, 4, nameof(calibrationPlanes));

            for (int i = 0; i < calibrationPlanes.Count; i++)
            {
                if (!calibrationPlanes[i].IsValid)
                    throw new ArgumentException($"Calibration plane {i} is invalid.", nameof(calibrationPlanes));
            }

            var origin = FourPointCalibration(calibrationPlanes);
            Tcp = new(origin, tcp.XAxis, tcp.YAxis);
        }
    }

    static Point3d FourPointCalibration(IReadOnlyList<Plane> calibrationPlanes)
    {
        var p = calibrationPlanes;
        var calibrate = new Geometry.CircumcentreSolver(p[0].Origin, p[1].Origin, p[2].Origin, p[3].Origin);
        Point3d tcpOrigin = Point3d.Origin;

        foreach (Plane plane in calibrationPlanes)
        {
            _ = plane.RemapToPlaneSpace(calibrate.Center, out Point3d remappedPoint);
            tcpOrigin += remappedPoint;
        }

        tcpOrigin /= 4.0;
        return tcpOrigin;
    }

    public override string ToString() => $"Tool ({Name})";
}
