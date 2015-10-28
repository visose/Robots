using System;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Parameters;
using static System.Math;
using static Robots.Util;

namespace Robots.Grasshopper
{
    public class CreateTarget : GH_Component, IGH_VariableParameterComponent
    {
        IGH_Param targetParam = new TargetParameter() { Name = "Target", NickName = "T", Description = "Reference target", Optional = false };
        IGH_Param planeParam = new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false };
        IGH_Param jointParam = new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false };
        IGH_Param toolParam = new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true };
        IGH_Param motionParam = new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true };
        IGH_Param speedParam = new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true };
        IGH_Param zoneParam = new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true };
        IGH_Param commandParam = new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true };
        IGH_Param configParam = new Param_Integer() { Name = "RobConf", NickName = "F", Description = "Robot configuration", Optional = true };

        public CreateTarget() : base("Create target", "Target", "Creates a target", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{BC68DC2C-EED6-4717-9F49-80A2B21B75B6}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            Params.RegisterInputParam(planeParam, 0);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Target sourceTarget = null;
            GH_Plane plane = null;
            GH_String joints = null;
            GH_Tool tool = null;
            GH_String motionText = null;
            GH_Speed speed = null;
            GH_Zone zone = null;
            GH_Command command = null;
            GH_Integer config = null;

            bool hasTarget = Params.IsInputParam(targetParam);
            bool hasPosition = HasPosition();
            bool hasTool = Params.Input.Any(x => x.Name == "Tool");
            bool hasMotion = Params.Input.Any(x => x.Name == "Motion");
            bool hasSpeed = Params.Input.Any(x => x.Name == "Speed");
            bool hasZone = Params.Input.Any(x => x.Name == "Zone");
            bool hasCommand = Params.Input.Any(x => x.Name == "Command");
            bool hasConfig = Params.Input.Any(x => x.Name == "RobConf");

            if (hasTarget) if (!DA.GetData("Target", ref sourceTarget)) return;
            if (hasPosition && IsCartesian()) if (!DA.GetData("Plane", ref plane)) return;
            if (hasPosition && !IsCartesian()) if (!DA.GetData("Joints", ref joints)) return;
            if (hasTool) DA.GetData("Tool", ref tool);
            if (hasMotion) DA.GetData("Motion", ref motionText);
            if (hasSpeed) DA.GetData("Speed", ref speed);
            if (hasZone) DA.GetData("Zone", ref zone);
            if (hasCommand) DA.GetData("Command", ref command);
            if (hasConfig) DA.GetData("RobConf", ref config);

            var motion = Target.Motions.JointCartesian;
            if (motionText != null)
                motion = (Target.Motions)Enum.Parse(typeof(Target.Motions), motionText.Value);

            var jointsNumbers = new double[6];
            if (!IsCartesian())
            {
                string[] jointsText = joints.Value.Split(',');
                if (jointsText.Length != 6) return;

                for (int i = 0; i < 6; i++)
                    if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref jointsNumbers[i])) return;
            }

            List<Robots.Commands.ICommand> commands = null;
            if (command != null)
            {
                commands = new List<Robots.Commands.ICommand>();
                commands.Add(command.Value);
            }

            Target target;

            if (!hasTarget)
            {
                if (IsCartesian())
                    target = new Target(plane.Value, tool?.Value, motion, speed?.Value, zone?.Value, commands, (config == null) ? 0 : (Target.RobotConfigurations)config.Value);
                else
                    target = new Target(jointsNumbers, tool?.Value, speed?.Value, zone?.Value, commands);
            }
            else
            {
                target = sourceTarget.Value.Duplicate();
                if (hasPosition && IsCartesian()) target.Plane = plane.Value;
                if (hasPosition && !IsCartesian()) target.JointRotations = jointsNumbers;
                if (hasTool && tool != null) target.Tool = tool.Value;
                if (hasMotion && motionText != null) target.Motion = motion;
                if (hasSpeed && speed != null) target.Speed = speed.Value;
                if (hasZone && zone != null) target.Zone = zone.Value;
                if (hasCommand && command != null) { target.Commands.Clear(); target.Commands.Add(command.Value); }
                if (hasConfig && config != null) target.Configuration = (Target.RobotConfigurations)config.Value;
            }
            DA.SetData(0, new GH_Target(target));
        }

        #region menu override
        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Target input", AddTarget, true, Params.Input.Any(x => x.Name == "Target"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Position input", AddPosition, true, HasPosition());
            Menu_AppendItem(menu, "Cartesian target", SwitchCartesian, HasPosition(), IsCartesian());
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Tool input", AddTool, true, Params.Input.Any(x => x.Name == "Tool"));
            Menu_AppendItem(menu, "Motion input", AddMotion, true, Params.Input.Any(x => x.Name == "Motion"));
            Menu_AppendItem(menu, "Speed input", AddSpeed, true, Params.Input.Any(x => x.Name == "Speed"));
            Menu_AppendItem(menu, "Zone input", AddZone, true, Params.Input.Any(x => x.Name == "Zone"));
            Menu_AppendItem(menu, "Command input", AddCommand, true, Params.Input.Any(x => x.Name == "Command"));
            Menu_AppendItem(menu, "Config input", AddConfig, true, Params.Input.Any(x => x.Name == "RobConf"));
        }

        bool HasPosition() => Params.Input.Any(x => (x.Name == "Plane") || (x.Name == "Joints"));
        bool IsCartesian() => !Params.Input.Any(x => (x.Name == "Joints"));

        private void AddParam(IGH_Param parameter, int index)
        {
            if (Params.Input.Any(x => x.Name == parameter.Name))
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == parameter.Name), true);
            else
                Params.RegisterInputParam(parameter, index);

            Params.OnParametersChanged();
            ExpireSolution(true);
        }

        private void AddTarget(object sender, EventArgs e) => AddParam(targetParam, 0);

        private void SwitchCartesian(object sender, EventArgs e)
        {
            if (IsCartesian())
            {
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == "Plane"), true);
                Params.RegisterInputParam(jointParam, 1);
            }
            else
            {
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == "Joints"), true);
                Params.RegisterInputParam(planeParam, 1);
            }

            Params.OnParametersChanged();
            ExpireSolution(true);
        }

        private void AddPosition(object sender, EventArgs e)
        {
            if (IsCartesian())
                AddParam(planeParam, 1);
            else
                AddParam(jointParam, 1);
        }

        private void AddTool(object sender, EventArgs e) => AddParam(toolParam, 2);
        private void AddMotion(object sender, EventArgs e) => AddParam(motionParam, 3);
        private void AddSpeed(object sender, EventArgs e) => AddParam(speedParam, 4);
        private void AddZone(object sender, EventArgs e) => AddParam(zoneParam, 5);
        private void AddCommand(object sender, EventArgs e) => AddParam(commandParam, 6);
        private void AddConfig(object sender, EventArgs e) => AddParam(configParam, 7);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
        #endregion
    }

    public class DeconstructTarget : GH_Component, IGH_VariableParameterComponent
    {
        bool isCartesian = true;
        IGH_Param planeParam = new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false };
        IGH_Param jointParam = new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false };
        IGH_Param toolParam = new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true };
        IGH_Param motionParam = new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true };
        IGH_Param speedParam = new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true };
        IGH_Param zoneParam = new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true };
        IGH_Param commandParam = new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true };
        IGH_Param configParam = new Param_Integer() { Name = "RobConf", NickName = "F", Description = "Robot configuration", Optional = true };

        public DeconstructTarget() : base("Deconstruct target", "DeTarget", "Deconstructs a target", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{3252D880-59F9-4C9A-8A92-A6CD4C0BA591}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            Params.RegisterOutputParam(planeParam, 0);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool hasPosition = HasPosition();
            bool hasTool = Params.Output.Any(x => x.Name == "Tool");
            bool hasMotion = Params.Output.Any(x => x.Name == "Motion");
            bool hasSpeed = Params.Output.Any(x => x.Name == "Speed");
            bool hasZone = Params.Output.Any(x => x.Name == "Zone");
            bool hasCommand = Params.Output.Any(x => x.Name == "Command");
            bool hasConfig = Params.Output.Any(x => x.Name == "RobConf");


            GH_Target target = null;
            string jointsText = null;

            if (!DA.GetData("Target", ref target)) return;
            isCartesian = target.Value.IsCartesian;
            if (!isCartesian)
                jointsText = string.Join(",", target.Value.JointRotations.Select(x => $"{x:0.000}"));

            if (HasPosition())
            {
                if (!isCartesian)
                {
                    Params.UnregisterOutputParameter(planeParam, true);
                    Params.RegisterOutputParam(jointParam, 0);
                }
                else
                {
                    Params.UnregisterOutputParameter(jointParam, true);
                    Params.RegisterOutputParam(planeParam, 0);
                }

                Params.OnParametersChanged();
                ExpireSolution(true);
            }

            if (hasPosition && isCartesian) DA.SetData("Plane", new GH_Plane(target.Value.Plane));
            if (hasPosition && !isCartesian) DA.SetData("Joints", new GH_String(jointsText));
            if (hasTool && (target.Value.Tool != null)) DA.SetData("Tool", new GH_Tool(target.Value.Tool));
            if (hasMotion) DA.SetData("Motion", new GH_Integer((int)target.Value.Motion));
            if (hasSpeed && (target.Value.Speed != null)) DA.SetData("Speed", new GH_Speed(target.Value.Speed));
            if (hasZone && (target.Value.Zone != null)) DA.SetData("Zone", new GH_Zone(target.Value.Zone));
            if (hasCommand) DA.SetData("Command", new GH_Command(target.Value.Commands));
            if (hasConfig) DA.SetData("RobConf", new GH_Integer((int)target.Value.Configuration));
        }

        #region menu override
        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Position input", AddPosition, true, HasPosition());
            Menu_AppendItem(menu, "Tool input", AddTool, true, Params.Output.Any(x => x.Name == "Tool"));
            Menu_AppendItem(menu, "Motion input", AddMotion, true, Params.Output.Any(x => x.Name == "Motion"));
            Menu_AppendItem(menu, "Speed input", AddSpeed, true, Params.Output.Any(x => x.Name == "Speed"));
            Menu_AppendItem(menu, "Zone input", AddZone, true, Params.Output.Any(x => x.Name == "Zone"));
            Menu_AppendItem(menu, "Command input", AddCommand, true, Params.Output.Any(x => x.Name == "Command"));
            Menu_AppendItem(menu, "Config input", AddConfig, true, Params.Output.Any(x => x.Name == "RobConf"));
        }

        bool HasPosition() => Params.Output.Any(x => (x.Name == "Plane") || (x.Name == "Joints"));

        private void AddParam(IGH_Param parameter, int index)
        {
            if (Params.Output.Any(x => x.Name == parameter.Name))
                Params.UnregisterOutputParameter(Params.Output.First(x => x.Name == parameter.Name), true);
            else
                Params.RegisterOutputParam(parameter, index);

            Params.OnParametersChanged();
            ExpireSolution(true);
        }

        private void AddPosition(object sender, EventArgs e)
        {
            if (isCartesian)
                AddParam(planeParam, 1);
            else
                AddParam(jointParam, 1);
        }

        private void AddTool(object sender, EventArgs e) => AddParam(toolParam, 2);
        private void AddMotion(object sender, EventArgs e) => AddParam(motionParam, 3);
        private void AddSpeed(object sender, EventArgs e) => AddParam(speedParam, 4);
        private void AddZone(object sender, EventArgs e) => AddParam(zoneParam, 5);
        private void AddCommand(object sender, EventArgs e) => AddParam(commandParam, 6);
        private void AddConfig(object sender, EventArgs e) => AddParam(configParam, 7);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
        #endregion
    }

    public class TargetParameter : GH_PersistentParam<GH_Target>
    {
        public TargetParameter() : base("Target parameter", "Target", "This is a robot target", "Robots", "Parameters") { }
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


    public class SpeedParameter : GH_PersistentParam<GH_Speed>
    {
        public SpeedParameter() : base("Speed parameter", "Speed", "This is a robot speed", "Robots", "Parameters") { }
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

            double value = 0;
            if (GH_Convert.ToDouble_Secondary((source as GH_String).Value, ref value))
            {
                Value = new Speed(value);
                return true;
            }
            else
                return false;
        }
    }

    public class ZoneParameter : GH_PersistentParam<GH_Zone>
    {
        public ZoneParameter() : base("Zone parameter", "Zone", "This is an aproximation zone", "Robots", "Parameters") { }
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

}