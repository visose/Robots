using Rhino.Geometry;

namespace Robots;

public class CobotCellDoosan : CobotCell
{
    internal CobotCellDoosan(string name, RobotDoosan robot, IO io, Plane basePlane, Mesh? environment)
        : base(name, Manufacturers.Doosan, robot, io, basePlane, environment)
    {
    }

    public static Plane QuaternionToPlane(double x, double y, double z, double q1, double q2, double q3, double q4)
    {
        var point = new Point3d(x, y, z) * 1000.0;
        var quaternion = new Quaternion(q1, q2, q3, q4);
        return quaternion.ToPlane(point);
    }

    public override double[] PlaneToNumbers(Plane plane)
    {
        var t = plane.ToTransform();
        var e = t.ToEulerZYX();
        return e.ToArray();
    }

    public override Plane NumbersToPlane(double[] numbers)
    {
        var euler = new Vector6d(numbers);
        var t = euler.EulerZYXToTransform();
        return t.ToPlane();
    }

    internal override List<List<List<string>>> Code(Program program) =>
        new DrlPostProcessor(this, program).Code;

    internal override void SaveCode(IProgram program, string folder)
    {
        if (program.Code is null)
            throw new InvalidOperationException(" Program code not generated");

        string filePath = Path.Combine(folder, $"{program.Name}.drl");
        var code = program.Code[0][0];
        var joinedCode = string.Join("\r\n", code);
        File.WriteAllText(filePath, joinedCode);
    }
}
