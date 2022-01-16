using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_RobotSystem : GH_Goo<RobotSystem>, IGH_PreviewData
{
    public GH_RobotSystem() { }
    public GH_RobotSystem(GH_RobotSystem goo) { Value = goo.Value; }
    public GH_RobotSystem(RobotSystem native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_RobotSystem(this);
    public override bool IsValid => true;
    public override string TypeName => "RobotSystem";
    public override string TypeDescription => "RobotSystem";
    public override string ToString() => Value.ToString();

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case RobotSystem system:
                Value = system;
                return true;
            default:
                return false;
        }
    }

    public BoundingBox ClippingBox => Value.DisplayMesh.GetBoundingBox(true);

    public void DrawViewportMeshes(GH_PreviewMeshArgs args)
    {
        args.Pipeline.DrawMeshShaded(Value.DisplayMesh, args.Material);
    }

    public void DrawViewportWires(GH_PreviewWireArgs args)
    {
    }
}
