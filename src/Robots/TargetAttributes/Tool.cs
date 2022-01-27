using Rhino.Geometry;

namespace Robots;

public class Tool : TargetAttribute
{
    public static Tool Default { get; } = new Tool(Plane.WorldXY, "DefaultTool");

    public Plane Tcp { get; }
    public double Weight { get; }
    public Point3d Centroid { get; }
    public Mesh Mesh { get; }

    public Tool(Plane tcp, string name = "DefaultTool", double weight = 0, Point3d? centroid = null, Mesh? mesh = null, IList<Plane>? calibrationPlanes = null) : base(name)
    {
        Weight = weight;
        Centroid = (centroid is null) ? tcp.Origin : (Point3d)centroid;
        Mesh = mesh ?? new Mesh();

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

    Point3d FourPointCalibration(IList<Plane> calibrationPlanes)
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