using System;
using System.Linq;
using System.Collections.Generic;
using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System.Drawing;

namespace Robots.Grasshopper
{
    public class GH_Robot : GH_Goo<Robot>, IGH_PreviewData
    {
        public GH_Robot() { this.Value = null; }
        public GH_Robot(GH_Robot goo) { this.Value = goo.Value; }
        public GH_Robot(Robot native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Robot(this);
        public override bool IsValid => true;
        public override string TypeName => "Robot";
        public override string TypeDescription => "Robot";
        public override string ToString() => this.Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Robot)
            {
                Value = source as Robot;
                return true;
            }

            if (source is GH_String)
            {
                Value = Robot.LoadFromLibrary((source as GH_String).Value, Plane.WorldXY);
                return true;
            }
            return false;
        }

        public BoundingBox ClippingBox
        {
            get
            {
                var box = new BoundingBox();
                foreach (var mesh in Value.GetMeshes()) box.Union(mesh.GetBoundingBox(true));
                return box;
            }
        }

        public void DrawViewportMeshes(GH_PreviewMeshArgs args)
        {
            foreach (var mesh in Value.GetMeshes())
                args.Pipeline.DrawMeshShaded(mesh,args.Material);
 
        }

        public void DrawViewportWires(GH_PreviewWireArgs args)
        {
         //   foreach (var mesh in Value.GetMeshes())
         //       args.Pipeline.DrawMeshWires(mesh,args.Color);
        }

    }

   
    public class GH_Program : GH_Goo<Program>
    {
        public GH_Program() { this.Value = null; }
        public GH_Program(GH_Program goo) { this.Value = goo.Value; }
        public GH_Program(Program native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Program(this);
        public override bool IsValid => true;
        public override string TypeName => "Program";
        public override string TypeDescription => "Program";
        public override string ToString() => this.Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Program)
            {
                Value = source as Program;
                return true;
            }
            return false;
        }
        }

    public class GH_Target : GH_Goo<Target>
    {
        public GH_Target() { this.Value = Target.Default; }
        public GH_Target(GH_Target goo) { this.Value = goo.Value; }
        public GH_Target(Target native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Target(this);
        public override bool IsValid => true;
        public override string TypeName => "Target";
        public override string TypeDescription => "Target";
        public override string ToString() => this.Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Target)
            {
                Value = source as Target;
                return true;
            }

            if (source is GH_Point)
            {
                Value = new Target(new Plane((source as GH_Point).Value, Vector3d.XAxis, Vector3d.YAxis));
                return true;
            }

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
                    if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref joints[i])) return false;

                Value = new Target(joints);
                return true;
            }
            return false;
        }
    }

    public class GH_Tool : GH_Goo<Tool>
    {
        public GH_Tool() { this.Value = null; }
        public GH_Tool(GH_Tool goo) { this.Value = goo.Value; }
        public GH_Tool(Tool native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Tool(this);
        public override bool IsValid => true;
        public override string TypeName => "Tool";
        public override string TypeDescription => "Tool";
        public override string ToString() => this.Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Tool)
            {
                Value = source as Tool;
                return true;
            }
            return false;
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
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Speed)
            {
                Value = source as Speed;
                return true;
            }

            if (source is GH_Number)
            {
                Value = new Speed((source as GH_Number).Value);
                return true;
            }

            if (source is GH_String)
            {
                double value = 0;
                if (GH_Convert.ToDouble_Secondary((source as GH_String).Value, ref value))
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
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Zone)
            {
                Value = source as Zone;
                return true;
            }

            if (source is GH_Number)
            {
                Value = new Zone((source as GH_Number).Value);
                return true;
            }

            double value = 0;
            if (GH_Convert.ToDouble_Secondary(source, ref value))
            {
                Value = new Zone(value);
                return true;
            }
            else
                return false;
        }
    }

    public class GH_Command : GH_Goo<Robots.Commands.ICommand>
    {
        public GH_Command() { this.Value = null; }
        public GH_Command(GH_Command goo) { this.Value = goo.Value; }
        public GH_Command(Robots.Commands.ICommand native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Command(this);
        public override bool IsValid => true;
        public override string TypeName => "Command";
        public override string TypeDescription => "Command";
        public override string ToString() => this.Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            if (source is Robots.Commands.ICommand)
            {
                Value = source as Robots.Commands.ICommand;
                return true;
            }
            return false;
        }
    }
}
