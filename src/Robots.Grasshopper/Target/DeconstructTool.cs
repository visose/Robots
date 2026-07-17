namespace Robots.Grasshopper;

public class DeconstructTool() : Component(
    "Deconstruct Tool",
    "Extracts data from a tool or end effector.",
    "Components",
    "{55B259ED-4349-4024-A50B-688A7679B50B}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ToolParameter(), "Tool", "T", "Robot tool to deconstruct.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Tool name.", GH_ParamAccess.item);
        _ = pManager.AddPlaneParameter("TCP", "P", "Tool center point plane.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Weight", "W", "Tool weight in kg.", GH_ParamAccess.item);
        _ = pManager.AddPointParameter("Centroid", "C", "Tool center of mass point.", GH_ParamAccess.item);
        _ = pManager.AddMeshParameter("Mesh", "M", "Tool preview mesh.", GH_ParamAccess.item);
        _ = pManager.AddMeshParameter("Collision Mesh", "Cm", "Tool collision mesh.", GH_ParamAccess.item);
        _ = pManager.AddBooleanParameter("Use Controller", "U", "Whether the tool definition stored in the controller is used.", GH_ParamAccess.item);
        _ = pManager.AddIntegerParameter("Number", "#", "Controller tool number.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var tool = DA.Get<Tool>(0);

        _ = DA.SetData(0, tool.Name);
        _ = DA.SetData(1, tool.Tcp);
        _ = DA.SetData(2, tool.Weight);
        _ = DA.SetData(3, tool.Centroid);
        _ = DA.SetData(4, tool.Mesh);
        _ = DA.SetData(5, tool.CollisionMesh);
        _ = DA.SetData(6, tool.UseController);

        if (tool.Number is { } number)
            _ = DA.SetData(7, number);
    }
}
