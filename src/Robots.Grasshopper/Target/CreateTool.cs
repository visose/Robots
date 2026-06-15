using Rhino.Geometry;

namespace Robots.Grasshopper;

public class CreateTool() : Component(
    "Create Tool",
    "Creates a tool or end effector.",
    "Components",
    "{E59E634B-7AD5-4682-B2C1-F18B73AE05C6}",
    GH_Exposure.tertiary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Tool name.", GH_ParamAccess.item, "DefaultGHTool");
        _ = pManager.AddPlaneParameter("TCP", "P", "Tool center point plane.", GH_ParamAccess.item, Plane.WorldXY);
        _ = pManager.AddPlaneParameter("Calibration", "4", "Optional 4-point TCP calibration. Orient the tool in 4 different ways around the same point in space and input the 4 planes that correspond to the flange.", GH_ParamAccess.list);
        _ = pManager.AddNumberParameter("Weight", "W", "Tool weight in kg.", GH_ParamAccess.item, 0.0);
        _ = pManager.AddPointParameter("Centroid", "C", "Optional tool center of mass point.", GH_ParamAccess.item);
        _ = pManager.AddMeshParameter("Mesh", "M", "Optional tool preview mesh.", GH_ParamAccess.item);
        pManager[2].Optional = true;
        pManager[4].Optional = true;
        pManager[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new ToolParameter(), "Tool", "T", "Robot tool.", GH_ParamAccess.item);
        _ = pManager.AddPlaneParameter("TCP", "P", "Calculated TCP plane after optional 4-point calibration.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var tool = new Tool(DA.Get<Plane>(1), DA.Get<string>(0), DA.Get<double>(3), DA.MaybeValue<Point3d>(4), DA.Maybe<Mesh>(5), DA.MaybeList<Plane>(2));

        _ = DA.SetData(0, tool);
        _ = DA.SetData(1, tool.Tcp);
    }
}
