using Rhino.Geometry;

namespace Robots;

public class CobotCellFranka : CobotCell
{
    internal CobotCellFranka(string name, RobotFranka robot, IO io, Plane basePlane, Mesh? environment)
        : base(name, Manufacturers.FrankaEmika, robot, io, basePlane, environment)
    {
        Remote = new RemoteFranka();
        RobotJointCount = 7;
    }

    public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
    {
        var point = new Point3d(x, y, z) * 1000.0;
        var quaternion = new Quaternion(q1, q2, q3, q4);
        return quaternion.ToPlane(point);
    }

    public static double[] PlaneToQuaternion(Plane plane)
    {
        var q = plane.ToQuaternion();
        var origin = plane.Origin / 1000.0;
        return new double[] { origin.X, origin.Y, origin.Z, q.A, q.B, q.C, q.D };
    }

    public override double[] PlaneToNumbers(Plane plane) =>
        PlaneToQuaternion(plane);
    public override Plane NumbersToPlane(double[] numbers) =>
        QuaternionToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5], numbers[6]);

    internal override List<List<List<string>>> Code(Program program) =>
        new FrankxPostProcessor(this, program).Code;

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        string filePath = Path.Combine(folder, $"{program.Name}.py");
        var code = program.Code[0][0];
        var joinedCode = string.Join("\r\n", code);
        File.WriteAllText(filePath, joinedCode);
    }
}
