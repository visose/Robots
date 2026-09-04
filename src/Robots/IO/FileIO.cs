using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots;

public enum ElementType { RobotSystem, Tool, Frame }

public static class FileIO
{
    public static List<string> List(ElementType type) => RobotLibrary.List(type);

    public static RobotSystem ParseRobotSystem(string xml, Plane basePlane, IPostProcessor? postProcessor = null) =>
        RobotSystemParser.Parse(xml, basePlane, null, postProcessor);

    public static RobotSystem ParseRobotSystem(string xml, Plane basePlane, File3dm meshDoc, IPostProcessor? postProcessor = null) =>
        RobotSystemParser.Parse(xml, basePlane, meshDoc, postProcessor);

    public static Tool ParseTool(string xml, File3dm meshDoc) =>
        TargetAttributeParser.ParseTool(xml, meshDoc);

    public static RobotSystem LoadRobotSystem(string name, Plane basePlane, bool loadMeshes = true, IPostProcessor? postProcessor = null)
    {
        var (element, file) = RobotLibrary.Load(name, ElementType.RobotSystem);
        var meshDoc = loadMeshes ? MeshIO.ReadDocument(file) : null;
        return RobotSystemParser.Parse(element, basePlane, meshDoc, postProcessor);
    }

    public static Tool LoadTool(string name)
    {
        var (element, file) = RobotLibrary.Load(name, ElementType.Tool);
        return TargetAttributeParser.ParseTool(element, MeshIO.ReadDocument(file));
    }

    public static Frame LoadFrame(string name)
    {
        var (element, _) = RobotLibrary.Load(name, ElementType.Frame);
        return TargetAttributeParser.ParseFrame(element);
    }

    /// <summary>
    /// Default Win: C:\Users\userName\Documents\Robots
    /// Default Mac: /Users/userName/Robots
    /// </summary>
    public static string LocalLibraryPath => RobotLibrary.LocalPath;

    public static string OnlineLibraryPath => RobotLibrary.OnlinePath;
}
