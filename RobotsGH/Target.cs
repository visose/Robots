using GH_IO.Serialization;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Special;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;
using System;
using System.Drawing;
using System.Linq;
using static System.Math;

namespace Robots.Grasshopper
{
    public sealed class CreateTarget : GH_Component, IGH_VariableParameterComponent
    {
        public CreateTarget() : base("Create target", "Target", "Creates or modifies a target. Right click for additional inputs", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{BC68DC2C-EED6-4717-9F49-80A2B21B75B6}");
        protected override Bitmap Icon => Properties.Resources.iconCreateTarget;

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
            bool hasFrame = Params.Input.Any(x => x.Name == "Frame");
            bool hasExternal = Params.Input.Any(x => x.Name == "External");

            GH_Target sourceTarget = null;
            if (hasTarget) if (!DA.GetData("Target", ref sourceTarget)) return;

            double[] joints = null;
            var plane = new Plane();
            RobotConfigurations? configuration = null;
            Motions motion = Motions.Joint;
            Tool tool = null;
            Speed speed = null;
            Zone zone = null;
            Command command = null;
            Frame frame = null;
            double[] external = null;

            if (hasJoints)
            {
                GH_String jointsGH = null;
                if (!DA.GetData("Joints", ref jointsGH)) return;

                string[] jointsText = jointsGH.Value.Split(',');
                if (jointsText.Length != 6) return;

                joints = new double[6];

                for (int i = 0; i < 6; i++)
                    if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref joints[i])) return;
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is JointTarget jointTarget)
                    joints = jointTarget.Joints;
            }

            if (hasPlane)
            {
                GH_Plane planeGH = null;
                if (hasPlane && !DA.GetData("Plane", ref planeGH)) return;
                plane = planeGH.Value;
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is CartesianTarget cartesian)
                    plane = cartesian.Plane;
            }

            if (hasConfig)
            {
                GH_Integer configGH = null;
                if (hasConfig) DA.GetData("RobConf", ref configGH);
                configuration = (configGH == null) ? null : (RobotConfigurations?)configGH.Value;
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is CartesianTarget cartesian)
                    configuration = cartesian.Configuration;
            }

            if (hasMotion)
            {
                GH_String motionGH = null;
                DA.GetData("Motion", ref motionGH);
                motion = (motionGH == null) ? Motions.Joint : (Motions)Enum.Parse(typeof(Motions), motionGH.Value);
            }
            else if (sourceTarget != null)
            {
                if (sourceTarget.Value is CartesianTarget cartesian)
                    motion = cartesian.Motion;
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
                command = commandGH?.Value;
            }
            else if (sourceTarget != null)
            {
                command = sourceTarget.Value.Command;
            }

            if (hasFrame)
            {
                GH_Frame frameGH = null;
                DA.GetData("Frame", ref frameGH);
                frame = frameGH?.Value;
            }
            else if (sourceTarget != null)
            {
                frame = sourceTarget.Value.Frame;
            }

            if (hasExternal)
            {
                GH_String externalGH = null;
                if (!DA.GetData("External", ref externalGH))
                {
                    external = new double[0];
                }
                else
                {
                    string[] externalText = externalGH.Value.Split(',');
                    int length = externalText.Length;
                    external = new double[length];
                    for (int i = 0; i < length; i++)
                        if (!GH_Convert.ToDouble_Secondary(externalText[i], ref external[i])) return;
                }
            }
            else if (sourceTarget != null)
            {
                external = sourceTarget.Value.External;
            }

            Target target;

            bool localCartesian = isCartesian;

            if (hasTarget && !hasPlane && !hasJoints)
                localCartesian = sourceTarget.Value is CartesianTarget;

            if (localCartesian)
                target = new CartesianTarget(plane, configuration, motion, tool, speed, zone, command, frame, external);
            else
                target = new JointTarget(joints, tool, speed, zone, command, frame, external);

            if (sourceTarget != null)
                target.ExternalCustom = sourceTarget.Value.ExternalCustom;

            DA.SetData(0, new GH_Target(target));
        }

        // Variable inputs

        readonly IGH_Param[] parameters = new IGH_Param[11]
{
         new TargetParameter() { Name = "Target", NickName = "T", Description = "Reference target", Optional = false },
         new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
         new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
         new Param_Integer() { Name = "RobConf", NickName = "Cf", Description = "Robot configuration", Optional = true },
         new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
         new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
         new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
         new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Aproximation zone in mm", Optional = true },
         new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true },
         new FrameParameter() { Name = "Frame", NickName = "F", Description = "Base frame", Optional = true },
         new Param_String() { Name = "External", NickName = "E", Description = "External axis", Optional = true }
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
            Menu_AppendItem(menu, "Frame input", AddFrame, true, Params.Input.Any(x => x.Name == "Frame"));
            Menu_AppendItem(menu, "External input", AddExternal, true, Params.Input.Any(x => x.Name == "External"));
        }

        // Varible methods

        private void SwitchCartesian()
        {
            if (isCartesian)
            {
                Params.UnregisterInputParameter(Params.Input.Find(x => x.Name == "Plane"), true);
                Params.UnregisterInputParameter(Params.Input.Find(x => x.Name == "RobConf"), true);
                Params.UnregisterInputParameter(Params.Input.Find(x => x.Name == "Motion"), true);
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
            {
                Params.UnregisterInputParameter(Params.Input.First(x => x.Name == parameter.Name), true);
            }
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
        private void AddFrame(object sender, EventArgs e) => AddParam(9);
        private void AddExternal(object sender, EventArgs e) => AddParam(10);

        bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
        bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
        IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null;
        bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
        void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
    }

    public sealed class DeconstructTarget : GH_Component, IGH_VariableParameterComponent
    {
        public DeconstructTarget() : base("Deconstruct target", "DeTarget", "Deconstructs a target. Right click for additional outputs", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{3252D880-59F9-4C9A-8A92-A6CD4C0BA591}");
        protected override Bitmap Icon => Properties.Resources.iconDeconstructTarget;

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

            bool isCartesian = target.Value is CartesianTarget;
            // if (isTargetCartesian != isCartesian) SwitchCartesian();

            bool hasJoints = Params.Output.Any(x => x.Name == "Joints");
            bool hasPlane = Params.Output.Any(x => x.Name == "Plane");
            bool hasConfig = Params.Output.Any(x => x.Name == "RobConf");
            bool hasMotion = Params.Output.Any(x => x.Name == "Motion");
            bool hasTool = Params.Output.Any(x => x.Name == "Tool");
            bool hasSpeed = Params.Output.Any(x => x.Name == "Speed");
            bool hasZone = Params.Output.Any(x => x.Name == "Zone");
            bool hasCommand = Params.Output.Any(x => x.Name == "Command");
            bool hasFrame = Params.Output.Any(x => x.Name == "Frame");
            bool hasExternal = Params.Output.Any(x => x.Name == "External");

            if (hasJoints) DA.SetData("Joints", isCartesian ? null : new GH_String(string.Join(",", (target.Value as JointTarget).Joints.Select(x => $"{x:0.000}"))));
            if (hasPlane) DA.SetData("Plane", isCartesian ? new GH_Plane(((CartesianTarget)target.Value).Plane) : null);
            if (hasConfig) DA.SetData("RobConf", isCartesian ? ((CartesianTarget)target.Value).Configuration == null ? null : new GH_Integer((int)((CartesianTarget)target.Value).Configuration) : null);
            if (hasMotion) DA.SetData("Motion", isCartesian ? new GH_String(((CartesianTarget)target.Value).Motion.ToString()) : null);
            if (hasTool && (target.Value.Tool != null)) DA.SetData("Tool", new GH_Tool(target.Value.Tool));
            if (hasSpeed && (target.Value.Speed != null)) DA.SetData("Speed", new GH_Speed(target.Value.Speed));
            if (hasZone && (target.Value.Zone != null)) DA.SetData("Zone", new GH_Zone(target.Value.Zone));
            if (hasCommand) DA.SetData("Command", new GH_Command(target.Value.Command));
            if (hasFrame) DA.SetData("Frame", new GH_Frame(target.Value.Frame));
            if (hasExternal) DA.SetData("External", new GH_String(string.Join(",", target.Value.External.Select(x => $"{x:0.000}"))));
        }

        // Variable outputs

        //bool isCartesian = false;

        readonly IGH_Param[] parameters = new IGH_Param[10]
{
         new Param_String() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
         new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
         new Param_Integer() { Name = "RobConf", NickName = "Cf", Description = "Robot configuration", Optional = true },
         new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
         new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
         new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
         new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Approximation zone in mm", Optional = true },
         new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true },
         new FrameParameter() { Name = "Frame", NickName = "F", Description = "Base frame", Optional = true },
         new Param_String() { Name = "External", NickName = "E", Description = "External axes", Optional = true }
};

        // Menu items

        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Joints output", AddJoints, true, Params.Output.Any(x => x.Name == "Joints"));
            Menu_AppendItem(menu, "Plane output", AddPlane, true, Params.Output.Any(x => x.Name == "Plane"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Config output", AddConfig, true, Params.Output.Any(x => x.Name == "RobConf"));
            Menu_AppendItem(menu, "Motion output", AddMotion, true, Params.Output.Any(x => x.Name == "Motion"));
            Menu_AppendSeparator(menu);
            Menu_AppendItem(menu, "Tool output", AddTool, true, Params.Output.Any(x => x.Name == "Tool"));
            Menu_AppendItem(menu, "Speed output", AddSpeed, true, Params.Output.Any(x => x.Name == "Speed"));
            Menu_AppendItem(menu, "Zone output", AddZone, true, Params.Output.Any(x => x.Name == "Zone"));
            Menu_AppendItem(menu, "Command output", AddCommand, true, Params.Output.Any(x => x.Name == "Command"));
            Menu_AppendItem(menu, "Frame output", AddFrame, true, Params.Output.Any(x => x.Name == "Frame"));
            Menu_AppendItem(menu, "External output", AddExternal, true, Params.Output.Any(x => x.Name == "External"));
        }

        private void AddParam(int index)
        {
            IGH_Param parameter = parameters[index];

            if (Params.Output.Any(x => x.Name == parameter.Name))
            {
                Params.UnregisterOutputParameter(Params.Output.First(x => x.Name == parameter.Name), true);
            }
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
        private void AddFrame(object sender, EventArgs e) => AddParam(8);
        private void AddExternal(object sender, EventArgs e) => AddParam(9);

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
                {
                    if (value is GH_Integer integer)
                    {
                        config += integer.Value;
                    }
                }
            }

            VolatileData.Clear();
            AddVolatileData(new GH_Path(0), 0, new GH_Integer(config));
        }
    }

    public class CreateFrame : GH_Component
    {
        public CreateFrame() : base("Create frame", "Frame", "Creates a frame or work plane.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{467237C8-08F5-4104-A553-8814AACAFE51}");
        protected override Bitmap Icon => Properties.Resources.iconFrame;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("Plane", "P", "Frame plane", GH_ParamAccess.item, Plane.WorldXY);
            pManager.AddIntegerParameter("Coupled mechanical group", "G", "Index of the mechanical group where the coupled mechanism or robot belongs, or -1 for no coupling.", GH_ParamAccess.item, -1);
            pManager.AddIntegerParameter("Coupled mechanism", "M", "Index of kinematically coupled mechanism or -1 for coupling of a robot in a multi robot cell. If input G is -1 this has no effect.", GH_ParamAccess.item, -1);
            pManager.AddTextParameter("Name", "N", "Optional name for the frame.", GH_ParamAccess.item);
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new FrameParameter(), "Frame", "F", "Frame", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Plane plane = null;
            int coupledGroup = -1;
            int coupledMechanism = -1;
            string name = null;

            if (!DA.GetData(0, ref plane)) { return; }
            if (!DA.GetData(1, ref coupledGroup)) { return; }
            if (!DA.GetData(2, ref coupledMechanism)) { return; }
            DA.GetData(3, ref name);

            var frame = new Frame(plane.Value, coupledMechanism, coupledGroup, name);
            DA.SetData(0, new GH_Frame(frame));
        }
    }

    public class CreateSpeed : GH_Component
    {
        public CreateSpeed() : base("Create speed", "Speed", "Creates a target speed.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.secondary;
        public override Guid ComponentGuid => new Guid("{BD11418C-74E1-4B13-BE1A-AF105906E1BC}");
        protected override Bitmap Icon => Properties.Resources.iconSpeed;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Translation", "T", "TCP translation speed", GH_ParamAccess.item, 100.0);
            pManager.AddNumberParameter("Rotation", "R", "TCP rotation and swivel speed", GH_ParamAccess.item, PI);
            pManager.AddNumberParameter("External translation", "Et", "External axes translation speed", GH_ParamAccess.item, 5000.0);
            pManager.AddNumberParameter("External rotation", "Er", "External axes rotation speed", GH_ParamAccess.item, PI * 6);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new SpeedParameter(), "Speed", "S", "Speed instance", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            double translationSpeed = 0, rotationSpeed = 0, translationExternal = 0, rotationExternal = 0;

            if (!DA.GetData(0, ref translationSpeed)) { return; }
            if (!DA.GetData(1, ref rotationSpeed)) { return; }
            if (!DA.GetData(2, ref translationExternal)) { return; }
            if (!DA.GetData(3, ref rotationExternal)) { return; }

            var speed = new Speed(translationSpeed, rotationSpeed, translationExternal, rotationExternal);
            DA.SetData(0, new GH_Speed(speed));
        }
    }
}