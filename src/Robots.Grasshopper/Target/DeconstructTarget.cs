﻿using System.Drawing;
using Grasshopper.Kernel.Parameters;

namespace Robots.Grasshopper;

public sealed class DeconstructTarget : GH_Component, IGH_VariableParameterComponent
{
    // Variable outputs
    readonly IGH_Param[] _parameters =
    [
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
    ];

    public DeconstructTarget() : base("Deconstruct target", "DeTarget", "Deconstructs a target. Right click for additional outputs.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{3252D880-59F9-4C9A-8A92-A6CD4C0BA591}");
    protected override Bitmap Icon => Util.GetIcon("iconDeconstructTarget");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        Params.RegisterOutputParam(_parameters[0]);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);

        FixJointsParams(0);
        FixJointsParams(9);
    }

    void FixJointsParams(int index)
    {
        var param = _parameters[index];
        var outputParam = Params.Output.FirstOrDefault(p => p.Name == param.Name);

        if (outputParam is null or JointsParameter)
            return;

        Params.UnregisterOutputParameter(outputParam, true);
        AddParam(index);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        Target? target = null;
        if (!DA.GetData("Target", ref target) || target is null) return;

        bool isCartesian = target is CartesianTarget;
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

        if (hasJoints) DA.SetData("Joints", isCartesian ? null : ((JointTarget)target).Joints);
        if (hasPlane) DA.SetData("Plane", isCartesian ? ((CartesianTarget)target).Plane : null);
        if (hasConfig && isCartesian)
        {
            var targetConfig = ((CartesianTarget)target).Configuration;

            if (targetConfig is not null)
                DA.SetData("RobConf", (int)targetConfig);
        }
        if (hasMotion) DA.SetData("Motion", isCartesian ? ((CartesianTarget)target).Motion.ToString() : null);
        if (hasTool && (target.Tool is not null)) DA.SetData("Tool", target.Tool);
        if (hasSpeed && (target.Speed is not null)) DA.SetData("Speed", target.Speed);
        if (hasZone && (target.Zone is not null)) DA.SetData("Zone", target.Zone);
        if (hasCommand) DA.SetData("Command", target.Command);
        if (hasFrame) DA.SetData("Frame", target.Frame);
        if (hasExternal) DA.SetData("External", target.External);
    }

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
        IGH_Param parameter = _parameters[index];

        if (Params.Output.Any(x => x.Name == parameter.Name))
        {
            Params.UnregisterOutputParameter(Params.Output.First(x => x.Name == parameter.Name), true);
        }
        else
        {
            int insertIndex = Params.Output.Count;
            for (int i = 0; i < Params.Output.Count; i++)
            {
                int otherIndex = Array.FindIndex(_parameters, x => x.Name == Params.Output[i].Name);
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
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => null!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }
}
