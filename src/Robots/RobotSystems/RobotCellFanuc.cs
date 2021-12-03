using Rhino.Geometry;
using static Robots.Util;

namespace Robots;

/// <summary>
/// Placeholder at the moment. Need to look into getting proper Fanuc data and conventions, etc...
/// </summary>
public class RobotCellFanuc : RobotCell
{
    internal RobotCellFanuc(string name, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.ABB, mechanicalGroups, io, basePlane, environment)
    {
        //Remote = new RemoteAbb(this);
    }

    public static Plane QuaternionToPlane(Point3d point, Quaternion quaternion)
    {
        quaternion.GetRotation(out Plane plane);
        plane.Origin = point;
        return plane;
    }

    public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
    {
        var point = new Point3d(x, y, z);
        var quaternion = new Quaternion(q1, q2, q3, q4);
        return QuaternionToPlane(point, quaternion);
    }

    public static double[] PlaneToQuaternion(Plane plane)
    {
        var q = GetRotation(plane);
        //   var q = Quaternion.Rotation(Plane.WorldXY, plane);
        return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, q.A, q.B, q.C, q.D };
    }

    public override double[] PlaneToNumbers(Plane plane) => PlaneToQuaternion(plane);
    public override Plane NumbersToPlane(double[] numbers) => QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);

    internal override void SaveCode(IProgram program, string folder)
    {
        // TODO: Implement...
        throw new NotImplementedException("Fanuc postprocessor not yet implemented.");
    }

    internal override List<List<List<string>>> Code(Program program) => new FanucPostProcessor(this, program).Code;
}
