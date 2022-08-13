using Autodesk.DesignScript.Runtime;
using Rhino.Geometry;

namespace Robots.Dynamo;

public class Robot
{
    // static
    /// <summary>
    /// Load a robot system
    /// </summary>
    /// <param name="name">Name of robot system</param>
    /// <returns>Robot system</returns>
    public static Robot ByName(string name)
    {
        var system = FileIO.LoadRobotSystem(name, Plane.WorldXY);
        return new(system);
    }

    // instance
    [IsVisibleInDynamoLibrary(false)]
    public RobotSystem System { get; }

    private Robot(RobotSystem system)
    {
        System = system;
    }

    public override string ToString() => $"RobotSystem(Name = {System.Name})";
}
