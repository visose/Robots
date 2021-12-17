using Rhino.Geometry;

namespace Robots;

public class Tool : TargetAttribute
{
    public static Tool Default { get; } = new Tool(Plane.WorldXY, "DefaultTool");

    Plane _tcp;
    public ref Plane Tcp => ref _tcp;
    public double Weight { get; set; }
    public Point3d Centroid { get; set; }
    public Mesh Mesh { get; set; }

    public Tool(Plane tcp, string? name = null, double weight = 0, Point3d? centroid = null, Mesh? mesh = null)
    {
        Name = name;
        Tcp = tcp;
        Weight = weight;
        Centroid = (centroid is null) ? tcp.Origin : (Point3d)centroid;
        Mesh = mesh ?? new Mesh();
    }

    public void FourPointCalibration(Plane a, Plane b, Plane c, Plane d)
    {
        var calibrate = new Geometry.CircumcentreSolver(a.Origin, b.Origin, c.Origin, d.Origin);
        Point3d tcpOrigin = Point3d.Origin;

        foreach (Plane plane in new Plane[] { a, b, c, d })
        {
            plane.RemapToPlaneSpace(calibrate.Center, out Point3d remappedPoint);
            tcpOrigin += remappedPoint;
        }

        tcpOrigin /= 4;
        Tcp = new Plane(tcpOrigin, Tcp.XAxis, Tcp.YAxis);
    }

    public override string ToString() => $"Tool ({Name})";
}