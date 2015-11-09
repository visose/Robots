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
        IGH_Param[] parameters = new IGH_Param[9]
        {
         new TargetParameter() { Name = "Target", NickName = "T", Description = "Reference target", Optional = false },
         new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
         new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
         new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
         new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
         new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
         new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true },
         new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true },
         new Param_Integer() { Name = "RobConf", NickName = "F", Description = "Robot configuration", Optional = true }
        };

        public CreateTarget() : base("Create target", "Target", "Creates a target", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{BC68DC2C-EED6-4717-9F49-80A2B21B75B6}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCreateTarget;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            Params.RegisterInputParam(parameters[1]);
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

            bool hasTarget = Params.Input.Any(x => x.Name == "Target");
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

        // Variable inputs
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

        private void AddParam(int index)
        {
            IGH_Param parameter = parameters[index];

            if (Params.Input.Any(x => x.Name == parameter.Name))
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == parameter.Name), true);
            else
            {
                int insertIndex = Params.Input.Count;
                for (int i=0;i<Params.Input.Count;i++)
                {
                    int otherIndex = Array.FindIndex(parameters, x => x.Name == Params.Input[i].Name);
                    if (otherIndex > index)
                    {
                        insertIndex = i;
                        break;
                    }
                    }
                Params.RegisterInputParam(parameter, insertIndex);
            }
            Params.OnParametersChanged();
            ExpireSolution(true);
        }

        private void AddTarget(object sender, EventArgs e) => AddParam(0);

        private void SwitchCartesian(object sender, EventArgs e)
        {
            if (IsCartesian())
            {
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == "Plane"), true);
                AddParam(2);
            }
            else
            {
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == "Joints"), true);
                AddParam(1);
            }

            Params.OnParametersChanged();
            ExpireSolution(true);
        }

        private void AddPosition(object sender, EventArgs e)
        {
            if (IsCartesian())
                AddParam(1);
            else
                AddParam(2);
        }
        private void AddTool(object sender, EventArgs e) => AddParam(3);
        private void AddMotion(object sender, EventArgs e) => AddParam(4);
        private void AddSpeed(object sender, EventArgs e) => AddParam(5);
        private void AddZone(object sender, EventArgs e) => AddParam(6);
        private void AddCommand(object sender, EventArgs e) => AddParam(7);
        private void AddConfig(object sender, EventArgs e) => AddParam(8);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
    }

    public class DeconstructTarget : GH_Component, IGH_VariableParameterComponent
    {
        bool isCartesian = true;

        IGH_Param[] parameters = new IGH_Param[8]
{
         new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
         new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
         new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
         new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
         new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
         new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true },
         new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true },
         new Param_Integer() { Name = "RobConf", NickName = "F", Description = "Robot configuration", Optional = true }
};

        public DeconstructTarget() : base("Deconstruct target", "DeTarget", "Deconstructs a target", "Robots", "Components")
        {

        }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{3252D880-59F9-4C9A-8A92-A6CD4C0BA591}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconDeconstructTarget;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            Params.RegisterOutputParam(parameters[0]);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool hasPosition = HasPosition();
            bool hasPlane = Params.Output.Any(x => x.Name == "Plane");
            bool hasJoints = Params.Output.Any(x => x.Name == "Joints");
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

            if (hasPlane && !isCartesian)
            {
                Params.UnregisterOutputParameter(parameters[0], true);
                AddParam(1);
            }

            if (hasJoints && isCartesian)
            {
                Params.UnregisterOutputParameter(parameters[1], true);
                AddParam(0);
            }

            if (hasPlane) DA.SetData("Plane", new GH_Plane(target.Value.Plane));
            if (hasJoints) DA.SetData("Joints", new GH_String(jointsText));
            if (hasTool && (target.Value.Tool != null)) DA.SetData("Tool", new GH_Tool(target.Value.Tool));
            if (hasMotion) DA.SetData("Motion", new GH_Integer((int)target.Value.Motion));
            if (hasSpeed && (target.Value.Speed != null)) DA.SetData("Speed", new GH_Speed(target.Value.Speed));
            if (hasZone && (target.Value.Zone != null)) DA.SetData("Zone", new GH_Zone(target.Value.Zone));
            if (hasCommand) DA.SetData("Command", new GH_Command(target.Value.Commands));
            if (hasConfig) DA.SetData("RobConf", new GH_Integer((int)target.Value.Configuration));
        }

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


        private void AddParam(int index)
        {
            IGH_Param parameter = parameters[index];

            if (Params.Output.Any(x => x.Name == parameter.Name))
                Params.UnregisterOutputParameter(Params.Output.First(x => x.Name == parameter.Name), true);
            else
            {
                int insertIndex = Params.Output.Count;
                for (int i = 0; i < Params.Output.Count; i++)
                {
                    int otherIndex = Array.FindIndex(parameters, x => x.Name == Params.Output[i].Name);
                    if (otherIndex > index)
                    {
                        insertIndex = i;
                        break;
                    }
                }
                Params.RegisterOutputParam(parameter, insertIndex);
            }
            Params.OnParametersChanged();
            ExpireSolution(true);
        }


        private void AddPosition(object sender, EventArgs e)
        {
            if (isCartesian)
                AddParam(0);
            else
                AddParam(1);
        }

        private void AddTool(object sender, EventArgs e) => AddParam(2);
        private void AddMotion(object sender, EventArgs e) => AddParam(3);
        private void AddSpeed(object sender, EventArgs e) => AddParam(4);
        private void AddZone(object sender, EventArgs e) => AddParam(5);
        private void AddCommand(object sender, EventArgs e) => AddParam(6);
        private void AddConfig(object sender, EventArgs e) => AddParam(7);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
    }

    public class CreateTool : GH_Component
    {
        public CreateTool() : base("Create tool", "Tool", "Creates a tool or end effector.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{E59E634B-7AD5-4682-B2C1-F18B73AE05C6}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconTool;

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Name", GH_ParamAccess.item, "DefaultTool");
            pManager.AddPlaneParameter("TCP", "P", "TCP plane", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddNumberParameter("Weight", "W", "Tool weight", GH_ParamAccess.item, 0.0);
            pManager.AddMeshParameter("Mesh", "M", "Tool geometry", GH_ParamAccess.item);

            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null;
            GH_Plane tcp = null;
            double weight = 0;
            GH_Mesh mesh = null;

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref tcp)) { return; }
            if (!DA.GetData(2, ref weight)) { return; }
            DA.GetData(3, ref mesh);

            var tool = new Tool(name, tcp.Value, weight, mesh?.Value);
            DA.SetData(0, new GH_Tool(tool));
        }
    }
}