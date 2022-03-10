using System.Drawing;
using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel.Parameters;
using Grasshopper.Kernel.Special;
using GH_IO.Serialization;

namespace Robots.Grasshopper;

public sealed class CreateTarget : GH_Component, IGH_VariableParameterComponent
{
    // Variable inputs
    readonly IGH_Param[] _parameters = new IGH_Param[11]
    {
            new TargetParameter() { Name = "Target", NickName = "T", Description = "Reference target", Optional = false },
            new JointsParameter() { Name = "Joints", NickName = "J", Description = "Joint rotations in radians", Optional = false },
            new Param_Plane() { Name = "Plane", NickName = "P", Description = "Target plane", Optional = false },
            new Param_Integer() { Name = "RobConf", NickName = "Cf", Description = "Robot configuration", Optional = true },
            new Param_String() { Name = "Motion", NickName = "M", Description = "Type of motion", Optional = true },
            new ToolParameter() { Name = "Tool", NickName = "T", Description = "Tool or end effector", Optional = true },
            new SpeedParameter() { Name = "Speed", NickName = "S", Description = "Speed of robot in mm/s", Optional = true },
            new ZoneParameter() { Name = "Zone", NickName = "Z", Description = "Approximation zone in mm", Optional = true },
            new CommandParameter() { Name = "Command", NickName = "C", Description = "Robot command", Optional = true },
            new FrameParameter() { Name = "Frame", NickName = "F", Description = "Base frame", Optional = true },
            new JointsParameter() { Name = "External", NickName = "E", Description = "External axes", Optional = true }
    };

    bool _isCartesian = true;

    public CreateTarget() : base("Create target", "Target", "Creates or modifies a target. Right click for additional inputs.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{BC68DC2C-EED6-4717-9F49-80A2B21B75B6}");
    protected override Bitmap Icon => Util.GetIcon("iconCreateTarget");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        Params.RegisterInputParam(_parameters[2]);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);

        FixJointsParams(1);
        FixJointsParams(10);
    }

    void FixJointsParams(int index)
    {
        var param = _parameters[index];
        var inputParam = Params.Input.FirstOrDefault(p => p.Name == param.Name);

        if (inputParam is null or JointsParameter)
            return;

        var sources = inputParam.Sources.ToList();

        Params.UnregisterInputParameter(inputParam, true);
        AddParam(index);

        var updatedParam = Params.Input.FirstOrDefault(p => p.Name == param.Name);

        foreach (var source in sources)
            updatedParam.AddSource(source);
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

        Target? sourceTarget = null;

        if (hasTarget)
        {
            if (!DA.GetData("Target", ref sourceTarget) || sourceTarget is null)
                return;
        }

        double[]? joints = null;
        var plane = new Plane();
        RobotConfigurations? configuration = null;
        Motions motion = Motions.Joint;
        Tool? tool = null;
        Speed? speed = null;
        Zone? zone = null;
        Command? command = null;
        Frame? frame = null;
        double[]? external = null;

        if (hasJoints)
        {
            if (!DA.GetData("Joints", ref joints) || joints is null)
                return;
        }
        else if (sourceTarget is not null)
        {
            if (sourceTarget is JointTarget jointTarget)
                joints = jointTarget.Joints;
        }

        if (hasPlane)
        {
            if (!DA.GetData("Plane", ref plane))
                return;
        }
        else if (sourceTarget is not null)
        {
            if (sourceTarget is CartesianTarget cartesian)
                plane = cartesian.Plane;
        }

        if (hasConfig)
        {
            int? config = null;
            DA.GetData("RobConf", ref config);
            configuration = (RobotConfigurations?)config;
        }
        else if (sourceTarget is not null)
        {
            if (sourceTarget is CartesianTarget cartesian)
                configuration = cartesian.Configuration;
        }

        if (hasMotion)
        {
            string motionText = "Joint";
            DA.GetData("Motion", ref motionText);
            Enum.TryParse(motionText, out motion);
        }
        else if (sourceTarget is not null)
        {
            if (sourceTarget is CartesianTarget cartesian)
                motion = cartesian.Motion;
        }

        if (hasTool)
        {
            DA.GetData("Tool", ref tool);
        }
        else if (sourceTarget is not null)
        {
            tool = sourceTarget.Tool;
        }

        if (hasSpeed)
        {
            DA.GetData("Speed", ref speed);
        }
        else if (sourceTarget is not null)
        {
            speed = sourceTarget.Speed;
        }

        if (hasZone)
        {
            DA.GetData("Zone", ref zone);
        }
        else if (sourceTarget is not null)
        {
            zone = sourceTarget.Zone;
        }

        if (hasCommand)
        {
            DA.GetData("Command", ref command);
        }
        else if (sourceTarget is not null)
        {
            command = sourceTarget.Command;
        }

        if (hasFrame)
        {
            DA.GetData("Frame", ref frame);
        }
        else if (sourceTarget is not null)
        {
            frame = sourceTarget.Frame;
        }

        if (hasExternal)
        {
            DA.GetData("External", ref external);
        }
        else if (sourceTarget is not null)
        {
            external = sourceTarget.External;
        }

        bool localCartesian = _isCartesian;

        if (hasTarget && !hasPlane && !hasJoints && sourceTarget is not null)
            localCartesian = sourceTarget is CartesianTarget;

        Target target;

        if (localCartesian)
        {
            target = new CartesianTarget(plane, configuration, motion, tool, speed, zone, command, frame, external);
        }
        else
        {
            if (joints is null)
                throw new ArgumentNullException(nameof(joints));

            target = new JointTarget(joints, tool, speed, zone, command, frame, external);
        }

        if (sourceTarget is not null)
            target.ExternalCustom = sourceTarget.ExternalCustom;

        DA.SetData(0, target);
    }

    public override bool Write(GH_IWriter writer)
    {
        writer.SetBoolean("IsCartesian", _isCartesian);
        return base.Write(writer);
    }

    public override bool Read(GH_IReader reader)
    {
        _isCartesian = reader.GetBoolean("IsCartesian");
        return base.Read(reader);
    }

    // Menu items
    protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Target input", AddTarget, true, Params.Input.Any(x => x.Name == "Target"));
        Menu_AppendSeparator(menu);
        Menu_AppendItem(menu, "Joint target", SwitchCartesianEvent, true, !_isCartesian);
        Menu_AppendItem(menu, "Joint input", AddJoints, !_isCartesian, Params.Input.Any(x => x.Name == "Joints"));
        Menu_AppendSeparator(menu);
        Menu_AppendItem(menu, "Cartesian target", SwitchCartesianEvent, true, _isCartesian);
        Menu_AppendItem(menu, "Plane input", AddPlane, _isCartesian, Params.Input.Any(x => x.Name == "Plane"));
        Menu_AppendItem(menu, "Configuration input", AddConfig, _isCartesian, Params.Input.Any(x => x.Name == "RobConf"));
        Menu_AppendItem(menu, "Motion input", AddMotion, _isCartesian, Params.Input.Any(x => x.Name == "Motion"));
        Menu_AppendSeparator(menu);
        Menu_AppendItem(menu, "Tool input", AddTool, true, Params.Input.Any(x => x.Name == "Tool"));
        Menu_AppendItem(menu, "Speed input", AddSpeed, true, Params.Input.Any(x => x.Name == "Speed"));
        Menu_AppendItem(menu, "Zone input", AddZone, true, Params.Input.Any(x => x.Name == "Zone"));
        Menu_AppendItem(menu, "Command input", AddCommand, true, Params.Input.Any(x => x.Name == "Command"));
        Menu_AppendItem(menu, "Frame input", AddFrame, true, Params.Input.Any(x => x.Name == "Frame"));
        Menu_AppendItem(menu, "External input", AddExternal, true, Params.Input.Any(x => x.Name == "External"));
    }

    // Variable methods
    private void SwitchCartesian()
    {
        if (_isCartesian)
        {
            Params.UnregisterInputParameter(Params.Input.Find(x => x.Name == "Plane"), true);
            Params.UnregisterInputParameter(Params.Input.Find(x => x.Name == "RobConf"), true);
            Params.UnregisterInputParameter(Params.Input.Find(x => x.Name == "Motion"), true);
            AddParam(1);
            _isCartesian = false;
        }
        else
        {
            Params.UnregisterInputParameter(Params.Input.FirstOrDefault(x => x.Name == "Joints"), true);
            AddParam(2);
            _isCartesian = true;
        }

        Params.OnParametersChanged();
        ExpireSolution(true);
    }
    private void SwitchCartesianEvent(object sender, EventArgs e) => SwitchCartesian();

    private void AddParam(int index)
    {
        IGH_Param parameter = _parameters[index];

        if (Params.Input.Any(x => x.Name == parameter.Name))
        {
            Params.UnregisterInputParameter(Params.Input.First(x => x.Name == parameter.Name), true);
        }
        else
        {
            int insertIndex = Params.Input.Count;

            for (int i = 0; i < Params.Input.Count; i++)
            {
                int otherIndex = Array.FindIndex(_parameters, x => x.Name == Params.Input[i].Name);
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
        var parameter = _parameters[3];

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
        var parameter = _parameters[4];

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
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
}
