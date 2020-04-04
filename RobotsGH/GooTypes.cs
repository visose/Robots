using System;
using System.Linq;
using System.Collections.Generic;
using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System.Drawing;

namespace Robots.Grasshopper
{
    public class GH_Toolpath : GH_Goo<IToolpath>
    {
        public GH_Toolpath() { Value = new SimpleToolpath(); }
        public GH_Toolpath(IToolpath native) { Value = native; }
        public override IGH_Goo Duplicate() => new GH_Toolpath(Value);
        public override bool IsValid => true;
        public override string TypeName => "Toolpath";
        public override string TypeDescription => "Toolpath";
        public override string ToString()
        {
            switch (Value.Targets)
            {
                case IList<Target> _:
                    var targets = Value.Targets as IList<Target>;
                    return $"Toolpath with ({targets.Count} targets)";
                case Target target:
                    return target.ToString();
                default:
                    return "Toolpath";
            }
        }
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case GH_Target target:
                    Value = target?.Value;
                    return true;
                case IToolpath toolpath:
                    Value = toolpath;
                    return true;
            }
            return false;
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
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Program program:
                    Value = program;
                    return true;
                default:
                    return false;
            }
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
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Target target:
                    Value = target;
                    return true;
                case GH_Point point:
                    Value = new CartesianTarget(new Plane(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                    return true;
                case GH_Plane plane:
                    Value = new CartesianTarget(plane.Value);
                    return true;
                case GH_String text:
                    {
                        string[] jointsText = text.Value.Split(',');
                        if (jointsText.Length != 6) return false;

                        var joints = new double[6];
                        for (int i = 0; i < 6; i++)
                            if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref joints[i])) return false;

                        Value = new JointTarget(joints);
                        return true;
                    }
            }

            return false;
        }
    }

    public class GH_Tool : GH_Goo<Tool>
    {
        public GH_Tool() { this.Value = Tool.Default; }
        public GH_Tool(GH_Tool goo) { this.Value = goo.Value; }
        public GH_Tool(Tool native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Tool(this);
        public override bool IsValid => true;
        public override string TypeName => "Tool";
        public override string TypeDescription => "Tool";
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Tool tool:
                    Value = tool;
                    return true;
                default:
                    return false;
            }
        }
    }

    public class GH_Speed : GH_Goo<Speed>
    {
        public GH_Speed() { this.Value = Speed.Default; }
        public GH_Speed(GH_Speed goo) { this.Value = goo.Value; }
        public GH_Speed(Speed native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Speed(this);
        public override bool IsValid => true;
        public override string TypeName => "Speed";
        public override string TypeDescription => "Speed";
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Speed speed:
                    Value = speed;
                    return true;
                case GH_Number number:
                    Value = new Speed(number.Value);
                    return true;
                case GH_String text:
                    {
                        string[] texts = text.Value.Split(',');
                        double[] values = new double[texts.Length];

                        for (int i = 0; i < texts.Length; i++)
                            if (!GH_Convert.ToDouble_Secondary(texts[i], ref values[i])) return false;

                        if (texts.Length == 1)
                        {
                            Value = new Speed(values[0]);
                            return true;
                        }
                        else if (texts.Length == 2)
                        {
                            Value = new Speed(values[0], values[1]);
                            return true;
                        }

                        break;
                    }
            }
            return false;
        }
    }

    public class GH_Zone : GH_Goo<Zone>
    {
        public GH_Zone() { this.Value = Zone.Default; }
        public GH_Zone(GH_Zone goo) { this.Value = goo.Value; }
        public GH_Zone(Zone native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Zone(this);
        public override bool IsValid => true;
        public override string TypeName => "Zone";
        public override string TypeDescription => "Zone";
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Zone zone:
                    Value = zone;
                    return true;
                case GH_Number number:
                    Value = new Zone(number.Value);
                    return true;
                case GH_String text:
                    {
                        string[] texts = text.Value.Split(',');
                        double[] values = new double[texts.Length];

                        for (int i = 0; i < texts.Length; i++)
                            if (!GH_Convert.ToDouble_Secondary(texts[i], ref values[i])) return false;

                        if (texts.Length == 1)
                        {
                            Value = new Zone(values[0]);
                            return true;
                        }
                        else if (texts.Length == 2)
                        {
                            Value = new Zone(values[0], values[1]);
                            return true;
                        }

                        break;
                    }
            }

            double value = 0;
            if (GH_Convert.ToDouble_Secondary(source, ref value))
            {
                Value = new Zone(value);
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    public class GH_Frame : GH_Goo<Frame>
    {
        public GH_Frame() { this.Value = Frame.Default; }
        public GH_Frame(GH_Frame goo) { this.Value = goo.Value; }
        public GH_Frame(Frame native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Frame(this);
        public override bool IsValid => true;
        public override string TypeName => "Frame";
        public override string TypeDescription => "Frame";
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Frame frame:
                    Value = frame;
                    return true;
                case GH_Plane plane:
                    Value = new Frame(plane.Value);
                    return true;
                case GH_Point point:
                    Value = new Frame(new Plane(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                    return true;
                default:
                    return false;
            }
        }
    }

    public class GH_Command : GH_Goo<Command>
    {
        public GH_Command() { this.Value = Command.Default; }
        public GH_Command(GH_Command goo) { this.Value = goo.Value; }
        public GH_Command(Command native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Command(this);
        public override bool IsValid => true;
        public override string TypeName => "Command";
        public override string TypeDescription => "Command";
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Command command:
                    Value = command;
                    return true;
                default:
                    return false;
            }
        }
    }

    public class GH_RobotSystem : GH_Goo<RobotSystem>, IGH_PreviewData
    {
        public GH_RobotSystem() { this.Value = null; }
        public GH_RobotSystem(GH_RobotSystem goo) { this.Value = goo.Value; }
        public GH_RobotSystem(RobotSystem native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_RobotSystem(this);
        public override bool IsValid => true;
        public override string TypeName => "RobotSystem";
        public override string TypeDescription => "RobotSystem";
        public override string ToString() => this.Value?.ToString();
        public override object ScriptVariable() => Value;

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
}
