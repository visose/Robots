using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Special;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using GH_IO.Serialization;

namespace Robots.Grasshopper
{
    public class CreateTarget : GH_Component, IGH_VariableParameterComponent
    {
        public CreateTarget() : base("Create target", "Target", "Creates or modifies a target. Right click for additional inputs", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{BC68DC2C-EED6-4717-9F49-80A2B21B75B6}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCreateTarget;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            Params.RegisterInputParam(parameters[2]);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            bool hasTarget = Params.Input.Any(x => x.Name == "Target");
            bool hasJoints = Params.Input.Any(x => x.Name == "Joints");
            bool hasPlane = Params.Input.Any(x => x.Name == "Plane");
            bool hasConfig = Params.Input.Any(x => x.Name == "RobConf");
            bool hasMotion = Params.Input.Any(x => x.Name == "Motion");
            bool hasTool = Params.Input.Any(x => x.Name == "Tool");
            bool hasSpeed = Params.Input.Any(x => x.Name == "Speed");
            bool hasZone = Params.Input.Any(x => x.Name == "Zone");
            bool hasCommand = Params.Input.Any(x => x.Name == "Command");

            GH_Target sourceTarget = null;
            if (hasTarget) if (!DA.GetData("Target", ref sourceTarget)) return;

            var joints = new double[6];
            var plane = new Plane();
            Target.RobotConfigurations? configuration = null;
            Target.Motions motion = Target.Motions.Joint;
            Tool tool = null;
            Speed speed = null;
            Zone zone = null;
            var commands = new List<Robots.Commands.ICommand>();


            if (hasJoints)
            {
                GH_String jointsGH = null;
                if (!DA.GetData("Joints", ref jointsGH)) return;

                string[] jointsText = jointsGH.Value.Split(',');
                if (jointsText.Length != 6) return;

                for (int i = 0; i < 6; i++)
                    if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref joints[i])) return;
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is JointTarget) joints = (sourceTarget.Value as JointTarget).Joints;
            }

            if (hasPlane)
            {
                GH_Plane planeGH = null;
                if (hasPlane) if (!DA.GetData("Plane", ref planeGH)) return;
                plane = planeGH.Value;
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is CartesianTarget) plane = (sourceTarget.Value as CartesianTarget).Plane;
            }

            if (hasConfig)
            {
                GH_Integer configGH = null;
                if (hasConfig) DA.GetData("RobConf", ref configGH);
                configuration = (configGH == null) ? null : (Target.RobotConfigurations?)configGH.Value;
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is CartesianTarget) configuration = (sourceTarget.Value as CartesianTarget).Configuration;
            }

            if (hasMotion)
            {
                GH_String motionGH = null;
                DA.GetData("Motion", ref motionGH);
                motion = (motionGH == null) ? Target.Motions.Joint : (Target.Motions)Enum.Parse(typeof(Target.Motions), motionGH.Value);
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is CartesianTarget) motion = (sourceTarget.Value as CartesianTarget).Motion;
            }

            if (hasTool)
            {
                GH_Tool toolGH = null;
                DA.GetData("Tool", ref toolGH);
                tool = toolGH?.Value;
            }
            else if (sourceTarget != null)
            {
                tool = sourceTarget.Value.Tool;
            }

            if (hasSpeed)
            {
                GH_Speed speedGH = null;
                DA.GetData("Speed", ref speedGH);
                speed = speedGH?.Value;
            }
            else if (sourceTarget != null)
            {
                speed = sourceTarget.Value.Speed;
            }

            if (hasZone)
            {
                GH_Zone zoneGH = null;
                DA.GetData("Zone", ref zoneGH);
                zone = zoneGH?.Value;
            }
            else if (sourceTarget != null)
            {
                zone = sourceTarget.Value.Zone;
            }

            if (hasCommand)
            {
                GH_Command commandGH = null;
                DA.GetData("Command", ref commandGH);
                if (commandGH != null) commands.Add(commandGH.Value);
            }
            else if (sourceTarget != null)
            {
                commands.AddRange(sourceTarget.Value.Commands.ToList());
            }

            Target target;

            if (isCartesian)
                target = new CartesianTarget(plane, configuration, motion, tool, speed, zone, commands);
            else
                target = new JointTarget(joints, tool, speed, zone, commands);

            DA.SetData(0, new GH_Target(target));
        }

        // Variable inputs

        IGH_Param[] parameters = new IGH_Param[9]
{
         new TargetParameter() { Name = "Target", NickName = "T", Description = "Reference target", Optional = false },
         new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
         new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
         new Param_Integer() { Name = "RobConf", NickName = "F", Description = "Robot configuration", Optional = true },
         new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
         new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
         new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
         new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true },
         new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true }
};

        bool isCartesian = true;

        public override bool Write(GH_IWriter writer)
        {
            writer.SetBoolean("IsCartesian", isCartesian);
            return base.Write(writer);
        }

        public override bool Read(GH_IReader reader)
        {
            isCartesian = reader.GetBoolean("IsCartesian");
            return base.Read(reader);
        }

        // Menu items

        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Target input", AddTarget, true, Params.Input.Any(x => x.Name == "Target"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Joint target", SwitchCartesianEvent, true, !isCartesian);
            Menu_AppendItem(menu, "Joint input", AddJoints, !isCartesian, Params.Input.Any(x => x.Name == "Joints"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Cartesian target", SwitchCartesianEvent, true, isCartesian);
            Menu_AppendItem(menu, "Plane input", AddPlane, isCartesian, Params.Input.Any(x => x.Name == "Plane"));
            Menu_AppendItem(menu, "Configuration input", AddConfig, isCartesian, Params.Input.Any(x => x.Name == "RobConf"));
            Menu_AppendItem(menu, "Motion input", AddMotion, isCartesian, Params.Input.Any(x => x.Name == "Motion"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Tool input", AddTool, true, Params.Input.Any(x => x.Name == "Tool"));
            Menu_AppendItem(menu, "Speed input", AddSpeed, true, Params.Input.Any(x => x.Name == "Speed"));
            Menu_AppendItem(menu, "Zone input", AddZone, true, Params.Input.Any(x => x.Name == "Zone"));
            Menu_AppendItem(menu, "Command input", AddCommand, true, Params.Input.Any(x => x.Name == "Command"));
        }

        // Varible methods

        private void SwitchCartesian()
        {
            if (isCartesian)
            {
                Params.UnregisterInputParameter(Params.Input.FirstOrDefault(x => x.Name == "Plane"), true);
                Params.UnregisterInputParameter(Params.Input.FirstOrDefault(x => x.Name == "RobConf"), true);
                Params.UnregisterInputParameter(Params.Input.FirstOrDefault(x => x.Name == "Motion"), true);
                AddParam(1);
                isCartesian = false;

            }
            else
            {
                Params.UnregisterInputParameter(Params.Input.FirstOrDefault(x => x.Name == "Joints"), true);
                AddParam(2);
                isCartesian = true;
            }

            Params.OnParametersChanged();
            ExpireSolution(true);
        }
        private void SwitchCartesianEvent(object sender, EventArgs e) => SwitchCartesian();

        private void AddParam(int index)
        {
            IGH_Param parameter = parameters[index];

            if (Params.Input.Any(x => x.Name == parameter.Name))
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == parameter.Name), true);
            else
            {
                int insertIndex = Params.Input.Count;
                for (int i = 0; i < Params.Input.Count; i++)
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
        private void AddJoints(object sender, EventArgs e) => AddParam(1);

        private void AddPlane(object sender, EventArgs e) => AddParam(2);
        private void AddConfig(object sender, EventArgs e)
        {
            AddParam(3);
            var parameter = parameters[3];

            if (Params.Input.Any(x => x.Name == parameter.Name))
            {
                var configParm = new ConfigParam();
                configParm.CreateAttributes();
                configParm.Attributes.Pivot = new PointF(parameter.Attributes.InputGrip.X - 110, parameter.Attributes.InputGrip.Y - 33);
                configParm.ListItems.Clear();
                configParm.ListMode = GH_ValueListMode.CheckList;
                configParm.ListItems.Add(new GH_ValueListItem("Shoulder", "1"));
                configParm.ListItems.Add(new GH_ValueListItem("Elbow", "2"));
                configParm.ListItems.Add(new GH_ValueListItem("Wrist", "4"));
                Instances.ActiveCanvas.Document.AddObject(configParm, false);
                parameter.AddSource(configParm);
                parameter.CollectData();

                ExpireSolution(true);
            }
        }
        private void AddMotion(object sender, EventArgs e)
        {
            AddParam(4);
            var parameter = parameters[4];

            if (Params.Input.Any(x => x.Name == parameter.Name))
            {
                var valueList = new GH_ValueList();
                valueList.CreateAttributes();
                valueList.Attributes.Pivot = new PointF(parameter.Attributes.InputGrip.X - 130, parameter.Attributes.InputGrip.Y - 11);
                valueList.ListItems.Clear();
                valueList.ListItems.Add(new GH_ValueListItem("Joint", "\"Joint\""));
                valueList.ListItems.Add(new GH_ValueListItem("Linear", "\"Linear\""));
                Instances.ActiveCanvas.Document.AddObject(valueList, false);
                parameter.AddSource(valueList);
                parameter.CollectData();
                ExpireSolution(true);
            }
        }

        private void AddTool(object sender, EventArgs e) => AddParam(5);
        private void AddSpeed(object sender, EventArgs e) => AddParam(6);
        private void AddZone(object sender, EventArgs e) => AddParam(7);
        private void AddCommand(object sender, EventArgs e) => AddParam(8);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
    }


    public class DeconstructTarget : GH_Component, IGH_VariableParameterComponent
    {
        public DeconstructTarget() : base("Deconstruct target", "DeTarget", "Deconstructs a target. Right click for additional outputs", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{3252D880-59F9-4C9A-8A92-A6CD4C0BA591}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconDeconstructTarget;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            Params.RegisterOutputParam(parameters[0]);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Target target = null;
            if (!DA.GetData("Target", ref target)) return;

            bool isTargetCartesian = target.Value is CartesianTarget;
            if (isTargetCartesian != isCartesian) SwitchCartesian();

            bool hasJoints = Params.Output.Any(x => x.Name == "Joints");
            bool hasPlane = Params.Output.Any(x => x.Name == "Plane");
            bool hasConfig = Params.Output.Any(x => x.Name == "RobConf");
            bool hasMotion = Params.Output.Any(x => x.Name == "Motion");
            bool hasTool = Params.Output.Any(x => x.Name == "Tool");
            bool hasSpeed = Params.Output.Any(x => x.Name == "Speed");
            bool hasZone = Params.Output.Any(x => x.Name == "Zone");
            bool hasCommand = Params.Output.Any(x => x.Name == "Command");

            if (hasJoints) DA.SetData("Joints", new GH_String(string.Join(",", (target.Value as JointTarget).Joints.Select(x => $"{x:0.000}"))));
            if (hasPlane) DA.SetData("Plane", new GH_Plane((target.Value as CartesianTarget).Plane));
            if (hasConfig) DA.SetData("RobConf", (target.Value as CartesianTarget).Configuration == null ? null : new GH_Integer((int)(target.Value as CartesianTarget).Configuration));
            if (hasMotion) DA.SetData("Motion", new GH_String((target.Value as CartesianTarget).Motion.ToString()));
            if (hasTool && (target.Value.Tool != null)) DA.SetData("Tool", new GH_Tool(target.Value.Tool));
            if (hasSpeed && (target.Value.Speed != null)) DA.SetData("Speed", new GH_Speed(target.Value.Speed));
            if (hasZone && (target.Value.Zone != null)) DA.SetData("Zone", new GH_Zone(target.Value.Zone));
            if (hasCommand) DA.SetData("Command", new GH_Command(target.Value.Commands));
        }

        // Variable outputs

        bool isCartesian = false;

        IGH_Param[] parameters = new IGH_Param[8]
{
         new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
         new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
         new Param_Integer() { Name = "RobConf", NickName = "F", Description = "Robot configuration", Optional = true },
         new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
         new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
         new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
         new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true },
         new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true }
};

        // Menu items

        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Joints output", AddJoints, !isCartesian, Params.Output.Any(x => x.Name == "Joints"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Plane output", AddPlane, isCartesian, Params.Output.Any(x => x.Name == "Plane"));
            Menu_AppendItem(menu, "Config output", AddConfig, isCartesian, Params.Output.Any(x => x.Name == "RobConf"));
            Menu_AppendItem(menu, "Motion output", AddMotion, isCartesian, Params.Output.Any(x => x.Name == "Motion"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Tool output", AddTool, true, Params.Output.Any(x => x.Name == "Tool"));
            Menu_AppendItem(menu, "Speed output", AddSpeed, true, Params.Output.Any(x => x.Name == "Speed"));
            Menu_AppendItem(menu, "Zone output", AddZone, true, Params.Output.Any(x => x.Name == "Zone"));
            Menu_AppendItem(menu, "Command output", AddCommand, true, Params.Output.Any(x => x.Name == "Command"));
        }

        // Varible methods

        private void SwitchCartesian()
        {
            if (isCartesian)
            {
                Params.UnregisterOutputParameter(Params.Output.FirstOrDefault(x => x.Name == "Plane"), true);
                Params.UnregisterOutputParameter(Params.Output.FirstOrDefault(x => x.Name == "RobConf"), true);
                Params.UnregisterOutputParameter(Params.Output.FirstOrDefault(x => x.Name == "Motion"), true);
                isCartesian = false;
            }
            else
            {
                Params.UnregisterOutputParameter(Params.Output.FirstOrDefault(x => x.Name == "Joints"), true);
                isCartesian = true;
            }

            Params.OnParametersChanged();
            ExpireSolution(true);
        }
        private void SwitchCartesianEvent(object sender, EventArgs e) => SwitchCartesian();

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

        private void AddJoints(object sender, EventArgs e) => AddParam(0);
        private void AddPlane(object sender, EventArgs e) => AddParam(1);
        private void AddConfig(object sender, EventArgs e) => AddParam(2);
        private void AddMotion(object sender, EventArgs e) => AddParam(3);
        private void AddTool(object sender, EventArgs e) => AddParam(4);
        private void AddSpeed(object sender, EventArgs e) => AddParam(5);
        private void AddZone(object sender, EventArgs e) => AddParam(6);
        private void AddCommand(object sender, EventArgs e) => AddParam(7);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
    }


    public class ConfigParam : GH_ValueList
    {
        public override string Name => "Flag fields";
        public override string Description => "Modified value list parameter for flag fields";
        public override Guid ComponentGuid => new Guid("{0381B555-BF9C-4D68-8E5C-10B2FCB16F30}");
        public override GH_Exposure Exposure => GH_Exposure.hidden;

        protected override void OnVolatileDataCollected()
        {
            int config = 0;
            if (VolatileDataCount > 0)
            {
                var values = VolatileData.get_Branch(0);

                foreach (var value in values)
                    if (value is GH_Integer)
                        config += (value as GH_Integer).Value;
            }

            VolatileData.Clear();
            AddVolatileData(new GH_Path(0), 0, new GH_Integer(config));
        }
    }
}