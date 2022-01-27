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
    }

    public override double[] PlaneToNumbers(Plane plane) => throw NotImplemented();

    public override Plane NumbersToPlane(double[] numbers) => throw NotImplemented();

    internal override void SaveCode(IProgram program, string folder)
    {
        // TODO: Implement...
        throw NotImplemented();
    }

    NotImplementedException NotImplemented() => new NotImplementedException(" Fanuc post-processor not yet implemented.");

    internal override List<List<List<string>>> Code(Program program) => new FanucPostProcessor(this, program).Code;
}
