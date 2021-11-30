using System;
using Rhino.Geometry;
using Grasshopper.Kernel;

namespace Robots.Grasshopper
{
    public class RobotSystemParameter : GH_Param<GH_RobotSystem>, IGH_PreviewObject
    {
        public RobotSystemParameter() : base("RobotSystem parameter", "RobotSystem", "This is a robot RobotSystem", "Robots", "Parameters", GH_ParamAccess.item) { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconRobotParam;
        public override Guid ComponentGuid => new Guid("{3DBDF573-248C-44B5-8D46-184657A56BCB}");
        public bool Hidden { get; set; }
        public bool IsPreviewCapable => true;
        public BoundingBox ClippingBox => base.Preview_ComputeClippingBox();
        public void DrawViewportWires(IGH_PreviewArgs args) => base.Preview_DrawMeshes(args);
        public void DrawViewportMeshes(IGH_PreviewArgs args) => base.Preview_DrawMeshes(args);
    }
}
