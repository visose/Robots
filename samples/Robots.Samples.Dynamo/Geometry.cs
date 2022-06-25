using Autodesk.DesignScript.Runtime;
using Rhino.Geometry;
using D = Autodesk.DesignScript.Geometry;

namespace Robots.Dynamo;

[IsVisibleInDynamoLibrary(false)]
public static class Geometry
{
    public static D.Plane ToDPlane(this ref Plane p)
    {
        var origin = p.Origin.ToDPoint();
        var xAxis = p.XAxis.ToDPoint();
        var yAxis = p.YAxis.ToDPoint();

        return D.Plane.ByOriginXAxisYAxis(origin, xAxis, yAxis);
    }

    public static Plane ToPlane(this D.Plane p)
    {
        var origin = p.Origin.ToPoint3d();
        var xAxis = p.XAxis.ToVector3d();
        var yAxis = p.YAxis.ToVector3d();

        return new(origin, xAxis, yAxis);
    }

    static D.Point ToDPoint(this Point3d p) => D.Point.ByCoordinates(p.X, p.Y, p.Z);
    static D.Vector ToDPoint(this Vector3d p) => D.Vector.ByCoordinates(p.X, p.Y, p.Z);

    static Point3d ToPoint3d(this D.Point p) => new(p.X, p.Y, p.Z);
    static Vector3d ToVector3d(this D.Vector p) => new(p.X, p.Y, p.Z);
}
