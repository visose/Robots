using Rhino.Geometry;

namespace Robots.Grasshopper;

public class RobotSystemParameter : GH_Param<GH_RobotSystem>, IGH_PreviewObject
{
    public RobotSystemParameter() : base("RobotSystem parameter", "RobotSystem", "This is a robot system", "Robots", "Parameters", GH_ParamAccess.item) { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconRobotParam");
    public override Guid ComponentGuid => new("{3DBDF573-248C-44B5-8D46-184657A56BCB}");
    protected override GH_RobotSystem PreferredCast(object data) =>
        data is RobotSystem cast ? new GH_RobotSystem(cast) : null!;
    public bool Hidden { get; set; }
    public bool IsPreviewCapable => true;
    public BoundingBox ClippingBox => Preview_ComputeClippingBox();
    public void DrawViewportWires(IGH_PreviewArgs args) => Preview_DrawMeshes(args);
    public void DrawViewportMeshes(IGH_PreviewArgs args) => Preview_DrawMeshes(args);
}