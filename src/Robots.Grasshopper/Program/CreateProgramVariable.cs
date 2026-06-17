using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class CreateProgramVariable() : Component(
    "Create Program",
    "Creates and checks a robot program.",
    "Components",
    ComponentIds.CreateProgram,
    GH_Exposure.quarternary)
    , IGH_VariableParameterComponent
{
    RobotSystem? _robotSystem;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Program name.", GH_ParamAccess.item, "DefaultProgram");
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system used by the program.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new ToolpathParameter(), "Targets", "T", "List of targets or toolpaths.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new CommandParameter(), "Init Commands", "C", "Optional list of commands that run at the start of the program.", GH_ParamAccess.list);
        _ = pManager.AddIntegerParameter("Multi-File Indices", "I", "Optional indices used to split the program into multiple files. Each index is the first target of an additional file.", GH_ParamAccess.list);
        _ = pManager.AddNumberParameter("Step Size", "S", "Distance in mm used to step through linear motions for error checking and simulation. Smaller values are more accurate but slower.", GH_ParamAccess.item, 1);
        pManager[2].Optional = true;
        pManager[3].Optional = true;
        pManager[4].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Code", "C", "Generated robot code.", GH_ParamAccess.tree);
        _ = pManager.AddNumberParameter("Duration", "D", "Program duration in seconds.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Warnings", "W", "Program warnings.", GH_ParamAccess.list);
        _ = pManager.AddTextParameter("Errors", "E", "Program errors.", GH_ParamAccess.list);
    }

    static int GetRobotCount(RobotSystem robotSystem)
    {
        if (robotSystem is not IndustrialSystem system)
            return 1;

        return system.MechanicalGroups.Count;
    }

    void SetTargetInputs(int count)
    {
        var inputs = Params.Input.Where(IsTargetInput).ToList();

        bool paramsChanged = false;

        foreach (var param in inputs.Skip(count))
        {
            _ = Params.UnregisterInputParameter(param, false);
            paramsChanged = true;
        }

        for (int i = 0; i < count; i++)
        {
            IGH_Param param;

            if (i < inputs.Count)
            {
                param = inputs[i];
            }
            else
            {
                param = new ToolpathParameter { Access = GH_ParamAccess.list, Optional = true };
                _ = Params.RegisterInputParam(param, i + 2);
                paramsChanged = true;
            }

            ConfigureTargetInput(param, i, count);
        }

        if (paramsChanged)
            Params.OnParametersChanged();
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var name = DA.Get<string>(InputIndex("Name"));
        var robotSystem = DA.Get<RobotSystem>(InputIndex("Robot System"));

        if (robotSystem != _robotSystem)
        {
            _robotSystem = robotSystem;
            SetTargetInputs(GetRobotCount(robotSystem));
        }

        var initCommandValues = MaybeList<Command>(DA, "Init Commands");
        var initCommands = initCommandValues.Length > 0 ? new Robots.Commands.Group(initCommandValues) : null;
        var multiFileIndices = MaybeList<int>(DA, "Multi-File Indices");
        var stepSizeIndex = InputIndex("Step Size");
        var stepSize = stepSizeIndex == -1 ? 1.0 : DA.Get<double>(stepSizeIndex);
        var toolpaths = ReadToolpaths(DA);

        var program = new Program(name, robotSystem, toolpaths, initCommands, multiFileIndices, stepSize);

        _ = DA.SetData(0, program);

        if (program.Code is not null)
            DA.SetTextTree(1, program.Code, DA.ParameterTargetPath(2));

        _ = DA.SetData(2, program.Duration);

        if (program.Warnings.Count > 0)
        {
            _ = DA.SetDataList(3, program.Warnings);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Program has warnings.");
        }

        if (program.Errors.Count > 0)
        {
            _ = DA.SetDataList(4, program.Errors);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Program has errors.");
        }
    }

    int InputIndex(string name) => Params.Input.FindIndex(p => p.Name == name);

    T[] MaybeList<T>(IGH_DataAccess DA, string name)
    {
        int index = InputIndex(name);
        return index == -1 ? [] : DA.MaybeList<T>(index);
    }

    int[] TargetInputIndices()
    {
        return [.. Params.Input.Select((param, index) => (param, index)).Where(x => IsTargetInput(x.param)).Select(x => x.index)];
    }

    static bool IsTargetInput(IGH_Param param) => param is ToolpathParameter or TargetParameter;

    IToolpath[] ReadToolpaths(IGH_DataAccess DA)
    {
        var indices = TargetInputIndices();
        var toolpaths = new IToolpath[indices.Length];

        for (int i = 0; i < indices.Length; i++)
        {
            int index = indices[i];
            var param = Params.Input[index];

            try
            {
                toolpaths[i] = ReadToolpath(DA, index, param);
            }
            catch (MissingInputException)
            {
                throw new RuntimeWarningException($"Input parameter {param.NickName} failed to collect data.");
            }
        }

        return toolpaths;
    }

    static SimpleToolpath ReadToolpath(IGH_DataAccess DA, int index, IGH_Param param)
    {
        if (param is ToolpathParameter)
            return new SimpleToolpath(DA.List<IToolpath>(index));

        return new(DA.List<Target>(index));
    }

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => false;
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => false;
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => default!;
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => false;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }

    internal IGH_Param EnsureTargetInputForUpgrade(int targetIndex)
    {
        int inputIndex = targetIndex + 2;

        while (TargetInputIndices().Length <= targetIndex)
        {
            var param = new ToolpathParameter { Access = GH_ParamAccess.list, Optional = true };
            _ = Params.RegisterInputParam(param, inputIndex);
        }

        var targets = TargetInputIndices();
        int count = targets.Length;

        for (int i = 0; i < targets.Length; i++)
            ConfigureTargetInput(Params.Input[targets[i]], i, count);

        return Params.Input[targets[targetIndex]];
    }

    static void ConfigureTargetInput(IGH_Param param, int index, int count)
    {
        int robot = index + 1;
        bool isOne = count == 1;

        param.Name = isOne ? "Targets" : $"Targets {robot}";
        param.NickName = isOne ? "T" : $"T{robot}";
        param.Description = isOne ? "List of targets or toolpaths." : $"List of targets or toolpaths for robot {robot}.";
    }
}
