using Grasshopper.Kernel;
using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace Robots.Grasshopper
{
    public class ToolpathParameter : GH_PersistentParam<GH_Toolpath>
    {
        public ToolpathParameter() : base("Toolpath", "Toolpath", "This is a robot toolpath.", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconProgramParam;
        public override Guid ComponentGuid => new Guid("{715AEDCE-14E8-400B-A226-9806FC3CB7B3}");

        protected override GH_GetterResult Prompt_Singular(ref GH_Toolpath value)
        {
            value = new GH_Toolpath();
            return GH_GetterResult.success;
        }

        protected override GH_GetterResult Prompt_Plural(ref List<GH_Toolpath> values)
        {
            values = new List<GH_Toolpath>();
            return GH_GetterResult.success;
        }
    }

    public class ProgramParameter : GH_PersistentParam<GH_Program>
    {
        public ProgramParameter() : base("Program parameter", "Program", "This is a robot program", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconProgramParam;
        public override Guid ComponentGuid => new Guid("{9C4F1BB6-5FA2-44DA-B7EA-421AF31DA054}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Program value)
        {
            value = new GH_Program();

            if (value.Value.HasCustomCode)
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, " Program contains custom code");

            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Program> values)
        {
            values = new List<GH_Program>();
            return GH_GetterResult.success;
        }
    }

    public class TargetParameter : GH_PersistentParam<GH_Target>
    {
        public TargetParameter() : base("Target parameter", "Target", "This is a robot target", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconTargetParam;
        public override Guid ComponentGuid => new Guid("{BEB590A9-905E-42ED-AB08-3E999EA94553}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Target value)
        {
            value = new GH_Target();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Target> values)
        {
            values = new List<GH_Target>();
            return GH_GetterResult.success;
        }
    }

    public class ToolParameter : GH_PersistentParam<GH_Tool>
    {
        public ToolParameter() : base("Tool parameter", "Tool", "This is a robot tool or end effector", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconToolParam;
        public override Guid ComponentGuid => new Guid("{073A02A6-2166-4387-9482-2EE3282E9209}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Tool value)
        {
            value = new GH_Tool();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Tool> values)
        {
            values = new List<GH_Tool>();
            return GH_GetterResult.success;
        }
    }

    public class SpeedParameter : GH_PersistentParam<GH_Speed>
    {
        public SpeedParameter() : base("Speed parameter", "Speed", "This is a robot speed", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconSpeedParam;
        public override Guid ComponentGuid => new Guid("{0B329813-13A0-48C4-B89A-65F289A4FF28}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Speed value)
        {
            value = new GH_Speed();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Speed> values)
        {
            values = new List<GH_Speed>();
            return GH_GetterResult.success;
        }
    }

    public class ZoneParameter : GH_PersistentParam<GH_Zone>
    {
        public ZoneParameter() : base("Zone parameter", "Zone", "This is a robot approximation zone", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconZoneParam;
        public override Guid ComponentGuid => new Guid("{458855D3-F671-4A50-BDA1-6AD3B7A5EC70}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Zone value)
        {
            value = new GH_Zone();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Zone> values)
        {
            values = new List<GH_Zone>();
            return GH_GetterResult.success;
        }
    }

    public class FrameParameter : GH_PersistentParam<GH_Frame>
    {
        public FrameParameter() : base("Frame parameter", "Frame", "This is a robot base frame", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconFrameParam;
        public override Guid ComponentGuid => new Guid("{6A012ECB-D161-4F93-BB60-D03391DF1A7C}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Frame value)
        {
            value = new GH_Frame();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Frame> values)
        {
            values = new List<GH_Frame>();
            return GH_GetterResult.success;
        }
    }

    public class CommandParameter : GH_PersistentParam<GH_Command>
    {
        public CommandParameter() : base("Command parameter", "Command", "This is a robot command", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCommandParam;
        public override Guid ComponentGuid => new Guid("{F5865990-90F3-4736-9AFF-4DD9ECEDA799}");

        protected override GH_GetterResult Prompt_Singular(ref GH_Command value)
        {
            value = new GH_Command();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Command> values)
        {
            values = new List<GH_Command>();
            return GH_GetterResult.success;
        }
    }

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
