using System;
using System.Collections.Generic;
using Rhino.Geometry;
using Grasshopper.Kernel;

namespace Robots.Grasshopper
{
    public class RobotSystemParameter : GH_PersistentParam<GH_RobotSystem>, IGH_PreviewObject
    {
        public RobotSystemParameter() : base("RobotSystem parameter", "RobotSystem", "This is a robot RobotSystem", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.primary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconRobotParam;
        public override Guid ComponentGuid => new Guid("{3DBDF573-248C-44B5-8D46-184657A56BCB}");

        protected override GH_GetterResult Prompt_Singular(ref GH_RobotSystem value)
        {
            value = new GH_RobotSystem();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_RobotSystem> values)
        {
            values = new List<GH_RobotSystem>();
            return GH_GetterResult.success;
        }

        public bool Hidden { get; set; }
        public bool IsPreviewCapable => true;
        public BoundingBox ClippingBox => base.Preview_ComputeClippingBox();
        public void DrawViewportWires(IGH_PreviewArgs args) => base.Preview_DrawMeshes(args);
        public void DrawViewportMeshes(IGH_PreviewArgs args) => base.Preview_DrawMeshes(args);
    }
}
