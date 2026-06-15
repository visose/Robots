using Rhino.Geometry;

namespace Robots.Grasshopper;

public class CommandParameter() : PersistentParam<Command, GH_Command>(
    "Command",
    "Robot command.",
    "{F5865990-90F3-4736-9AFF-4DD9ECEDA799}",
    GH_Exposure.tertiary);

public class FrameParameter() : PersistentParam<Frame, GH_Frame>(
    "Frame",
    "Robot frame.",
    "{6A012ECB-D161-4F93-BB60-D03391DF1A7C}",
    GH_Exposure.tertiary);

public class JointsParameter() : PersistentParam<double[], GH_Joints>(
    "Joints",
    "Joint values in radians.",
    "{2A47BF40-61D3-46D2-AA32-AF794C2F240B}",
    GH_Exposure.tertiary);

public class PostProcessorParameter() : Param<IPostProcessor, GH_PostProcessor>(
    "Post Processor",
    "Optional post processor override.",
    "12e20f7a-82c9-400c-bbee-c6ba46b78046");

public class ProgramParameter() : Param<IProgram, GH_Program>(
    "Program",
    "Robot program.",
    "{9C4F1BB6-5FA2-44DA-B7EA-421AF31DA054}",
    GH_Exposure.quarternary);

public class RobotSystemParameter() : Param<RobotSystem, GH_RobotSystem>(
    "Robot System",
    "Robot system definition.",
    "{3DBDF573-248C-44B5-8D46-184657A56BCB}"),
    IGH_PreviewObject
{
    public bool Hidden { get; set; }
    public bool IsPreviewCapable => true;
    public BoundingBox ClippingBox => Preview_ComputeClippingBox();
    public void DrawViewportWires(IGH_PreviewArgs args) => Preview_DrawMeshes(args);
    public void DrawViewportMeshes(IGH_PreviewArgs args) => Preview_DrawMeshes(args);
}

public class SpeedParameter() : PersistentParam<Speed, GH_Speed>(
    "Speed",
    "Robot speed settings.",
    "{0B329813-13A0-48C4-B89A-65F289A4FF28}",
    GH_Exposure.tertiary);

public class TargetParameter() : PersistentParam<Target, GH_Target>(
    "Target",
    "Robot target.",
    "{BEB590A9-905E-42ED-AB08-3E999EA94553}",
    GH_Exposure.secondary);

public class ToolParameter() : PersistentParam<Tool, GH_Tool>(
    "Tool",
    "Robot tool.",
    "{073A02A6-2166-4387-9482-2EE3282E9209}",
    GH_Exposure.tertiary);

public class ToolpathParameter() : PersistentParam<IToolpath, GH_Toolpath>(
    "Toolpath",
    "Robot toolpath.",
    "{715AEDCE-14E8-400B-A226-9806FC3CB7B3}",
    GH_Exposure.secondary);

public class ZoneParameter() : PersistentParam<Zone, GH_Zone>(
    "Zone",
    "Robot approximation zone.",
    "{458855D3-F671-4A50-BDA1-6AD3B7A5EC70}",
    GH_Exposure.tertiary);
