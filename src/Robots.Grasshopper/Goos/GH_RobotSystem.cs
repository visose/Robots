using Rhino.Geometry;

namespace Robots.Grasshopper;

public class GH_RobotSystem() : Goo<RobotSystem, GH_RobotSystem>("Robot System"), IGH_PreviewData
{
    public BoundingBox ClippingBox => Value?.DisplayMesh.GetBoundingBox(true) ?? BoundingBox.Empty;

    public void DrawViewportMeshes(GH_PreviewMeshArgs args)
    {
        if (Value is not null)
            args.Pipeline.DrawMeshShaded(Value.DisplayMesh, args.Material);
    }

    public void DrawViewportWires(GH_PreviewWireArgs args)
    {
    }
}
