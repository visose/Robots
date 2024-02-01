using Rhino.Geometry;

namespace Robots;

public class Tool : TargetAttribute
{
    public static Tool Default { get; } = new(Plane.WorldXY, "DefaultTool");

    public Plane Tcp { get; }
    public double Weight { get; }
    public Point3d Centroid { get; }
    public Mesh Mesh { get; }

    /// <summary>
    /// Specifies that it will use a tool that exists in the controller and does not need to be defined in the generated program.
    /// </summary>
    public bool UseController { get; }

    /// <summary>
    /// Used only in KUKA to load from the TOOL_DATA array.
    /// </summary>
    public int? Number { get; }

    public Tool(Plane tcp, string name = "DefaultTool", double weight = 0, Point3d? centroid = null, Mesh? mesh = null, IList<Plane>? calibrationPlanes = null, bool useController = false, int? number = null)
        : base(name)
    {
        Weight = weight;
        Centroid = (centroid is null) ? tcp.Origin : (Point3d)centroid;
        Mesh = mesh ?? FileIO.EmptyMesh;
        UseController = number is not null || useController;
        Number = number;

        if (number is not null && number.Value < 1)
            throw new ArgumentOutOfRangeException(nameof(number), " Tool number out of range.");

        if (calibrationPlanes is null || !calibrationPlanes.Any())
        {
            Tcp = tcp;
        }
        else
        {
            if (calibrationPlanes.Count != 4)
                throw new ArgumentException(" Calibration requires 4 planes.", nameof(calibrationPlanes));

            var origin = FourPointCalibration(calibrationPlanes);
            Tcp = new Plane(origin, tcp.XAxis, tcp.YAxis);
        }
    }

    static Point3d FourPointCalibration(IList<Plane> calibrationPlanes)
    {
        var p = calibrationPlanes;
        var calibrate = new Geometry.CircumcentreSolver(p[0].Origin, p[1].Origin, p[2].Origin, p[3].Origin);
        Point3d tcpOrigin = Point3d.Origin;

        foreach (Plane plane in calibrationPlanes)
        {
            plane.RemapToPlaneSpace(calibrate.Center, out Point3d remappedPoint);
            tcpOrigin += remappedPoint;
        }

        tcpOrigin /= 4.0;
        return tcpOrigin;
    }

    public override string ToString() => $"Tool ({Name})";
}
