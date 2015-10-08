using System;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using static Robots.Util;

namespace Robots.Grasshopper
{
    public class TargetComponent : GH_Component
    {
        public TargetComponent() : base("Target", "Target", "Target", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{BC68DC2C-EED6-4717-9F49-80A2B21B75B6}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Plane", GH_ParamAccess.item);
            pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
            pManager.AddTextParameter("Motion", "M", "Motion", GH_ParamAccess.item);
            pManager.AddParameter(new SpeedParameter(), "Speed", "S", "Speed", GH_ParamAccess.item);
            pManager.AddParameter(new ZoneParameter(), "Zone", "Z", "Zone", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Flips", "F", "Flips", GH_ParamAccess.item);
            pManager.AddParameter(new CommandParameter(), "Command", "C", "Command", GH_ParamAccess.item);

            for (int i = 1; i < 7; i++)
                pManager[i].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Plane plane = null;
            GH_Tool tool = null;
            GH_String inMotion = null;
            object inSpeed = null;
            object inZone = null;
            GH_Integer inFlips = null;
            GH_Command command = null;

            if (!DA.GetData(0, ref plane)) { return; }
            DA.GetData(1, ref tool);
            DA.GetData(2, ref inMotion);
            DA.GetData(3, ref inSpeed);
            DA.GetData(4, ref inZone);
            DA.GetData(5, ref inFlips);
            DA.GetData(6, ref command);


            var motion = Target.Motions.Linear;
            if (inMotion != null)
                motion = (Target.Motions)Enum.Parse(typeof(Target.Motions), inMotion.Value);

            Speed speed = inSpeed as Speed;
            if (speed == null)
            {
                GH_Number number = inSpeed as GH_Number;
                if (number != null)
                    speed = new Speed(number.Value);
            }

            Zone zone = inZone as Zone;
            if (zone == null)
            {
                GH_Number number = inZone as GH_Number;
                if (number != null)
                    zone = new Zone((inZone as GH_Number).Value);
            }

            int flips = 0;
            if (inFlips != null)
                flips = inFlips.Value;


            var commands = new Commands.List();
            if (command != null)
                commands.Add(command.Value);

            var target = new Target(plane.Value, tool?.Value, motion, speed, zone, flips, commands);
            DA.SetData(0, new GH_Target(target));
        }
    }


    public class TargetParameter : GH_PersistentParam<GH_Target>
    {
        public TargetParameter() : base("Target", "Target", "This is a target.", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => null;   // 24x24 pixels // Properties.Resources.bitmapparameter;
        public override System.Guid ComponentGuid => new Guid("{BEB590A9-905E-42ED-AB08-3E999EA94553}");
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

    public class GH_Target : GH_Goo<Target>
    {
        public GH_Target() { this.Value = null; }
        public GH_Target(GH_Target goo) { this.Value = goo.Value; }
        public GH_Target(Target native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Target(this);
        public override bool IsValid => true;
        public override string TypeName => "Target";
        public override string TypeDescription => "Target";
        public override string ToString() => this.Value.ToString();

        public override bool CastFrom(object source)
        {
            if (source is GH_Plane)
            {
                Value = new Target((source as GH_Plane).Value);
                return true;
            }

            if (source is GH_String)
            {
                string text = (source as GH_String).Value;
                string[] jointsText = text.Split(',');
                if (jointsText.Length != 6) return false;

                var joints = new double[6];
                for (int i = 0; i < 6; i++)
                {
                    if (!double.TryParse(jointsText[i], out joints[i])) return false;
                }
                Value = new Target(DegreesToRadians(joints));
                return true;
            }
            return false;
        }
    }


    public class SpeedParameter : GH_PersistentParam<GH_Speed>
    {
        public SpeedParameter() : base("Speed", "Speed", "This is a speed.", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => null;   // 24x24 pixels // Properties.Resources.bitmapparameter;
        public override System.Guid ComponentGuid => new Guid("{0B329813-13A0-48C4-B89A-65F289A4FF28}");
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

    public class GH_Speed : GH_Goo<Speed>
    {
        public GH_Speed() { this.Value = null; }
        public GH_Speed(GH_Speed goo) { this.Value = goo.Value; }
        public GH_Speed(Speed native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Speed(this);
        public override bool IsValid => true;
        public override string TypeName => "Speed";
        public override string TypeDescription => "Speed";
        public override string ToString() => this.Value.ToString();

        public override bool CastFrom(object source)
        {
            if (source is GH_Number)
            {
                Value = new Speed((source as GH_Number).Value);
                return true;
            }
            if (source is GH_Integer)
            {
                Value = new Speed((double)(source as GH_Integer).Value);
                return true;
            }
            if (source is GH_String)
            {
                double value;
                if (double.TryParse((source as GH_String).Value, out value))
                {
                    Value = new Speed(value);
                    return true;
                }
                else
                    return false;
            }
            return false;
        }
    }


    public class ZoneParameter : GH_PersistentParam<GH_Zone>
    {
        public ZoneParameter() : base("Zone", "Zone", "This is a Zone.", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        protected override System.Drawing.Bitmap Icon => null;   // 24x24 pixels // Properties.Resources.bitmapparameter;
        public override System.Guid ComponentGuid => new Guid("{458855D3-F671-4A50-BDA1-6AD3B7A5EC70}");
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

    public class GH_Zone : GH_Goo<Zone>
    {
        public GH_Zone() { this.Value = null; }
        public GH_Zone(GH_Zone goo) { this.Value = goo.Value; }
        public GH_Zone(Zone native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Zone(this);
        public override bool IsValid => true;
        public override string TypeName => "Zone";
        public override string TypeDescription => "Zone";
        public override string ToString() => this.Value.ToString();

        public override bool CastFrom(object source)
        {
            if (source is GH_Number)
            {
                Value = new Zone((source as GH_Number).Value);
                return true;
            }
            if (source is GH_Integer)
            {
                Value = new Zone((double)(source as GH_Integer).Value);
                return true;
            }
            if (source is GH_String)
            {
                double value;
                if (double.TryParse((source as GH_String).Value, out value))
                {
                    Value = new Zone(value);
                    return true;
                }
                else
                    return false;
            }
            return false;
        }
    }
}