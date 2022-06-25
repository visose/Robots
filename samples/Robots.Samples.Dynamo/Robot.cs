using Autodesk.DesignScript.Runtime;
using Rhino.Geometry;

namespace Robots.Dynamo;

public class Robot
{
    // static
    /// <summary>
    /// Load a robot system
    /// </summary>
    /// <param name="cellName">Name of robot system</param>
    /// <returns>Robot system</returns>
    public static Robot ByName(string cellName)
    {
        var cell = FileIO.LoadRobotSystem(cellName, Plane.WorldXY);
        return new(cell);
    }

    // instance
    [IsVisibleInDynamoLibrary(false)]
    public RobotSystem Cell { get; }

    private Robot(RobotSystem cell)
    {
        Cell = cell;
    }

    public override string ToString() => $"RobotSystem(Name = {Cell.Name})";
}