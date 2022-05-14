using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;

namespace Robots.Grasshopper;

public class CreateProgramVariable : GH_Component
{
    RobotSystem? _robotSystem;

    public CreateProgramVariable() : base("Create program", "Program", "Creates a program, checks for possible issues and fixes common mistakes.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.quarternary;
    public override Guid ComponentGuid => new("{7E7D6476-7667-493E-8BCD-88244BB16138}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconCreateProgram");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Program name", GH_ParamAccess.item, "DefaultProgram");
        pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system used in program", GH_ParamAccess.item);
        pManager.AddParameter(new ToolpathParameter(), "Targets", "T", "List of targets or tool-paths.", GH_ParamAccess.list);
        pManager.AddParameter(new CommandParameter(), "Init commands", "C", "Optional list of commands that will run at the start of the program", GH_ParamAccess.list);
        pManager.AddIntegerParameter("Multi-file indices", "I", "Optional list of indices to split the program into multiple files. The indices correspond to the first target of the additional files", GH_ParamAccess.list);
        pManager.AddNumberParameter("Step Size", "S", "Distance in mm to step through linear motions, used for error checking and program simulation. Smaller is more accurate but slower", GH_ParamAccess.item, 1);
        pManager[2].Optional = true;
        pManager[3].Optional = true;
        pManager[4].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        pManager.AddTextParameter("Code", "C", "Code", GH_ParamAccess.tree);
        pManager.AddNumberParameter("Duration", "D", "Program duration in seconds", GH_ParamAccess.item);
        pManager.AddTextParameter("Warnings", "W", "Warnings in program", GH_ParamAccess.list);
        pManager.AddTextParameter("Errors", "E", "Errors in program", GH_ParamAccess.list);
    }

    int GetRobotCount()
    {
        if (_robotSystem is not RobotCell cell)
            return 1;

        return cell.MechanicalGroups.Count;
    }

    void SetTargetInputs()
    {
        var count = GetRobotCount();
        var inputs = Params.Input.OfType<ToolpathParameter>().ToList();

        bool paramsChanged = false;

        foreach (var param in inputs.Skip(count))
        {
            Params.UnregisterInputParameter(param, false);
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
                Params.RegisterInputParam(param, i + 2);
                paramsChanged = true;
            }

            int rob = i + 1;
            bool isOne = count == 1;

            param.Name = isOne ? "Targets" : $"Targets {rob}";
            param.NickName = isOne ? "T" : $"T{rob}";
            param.Description = isOne ? "List of targets or tool-paths" : $"List of targets or tool-paths for robot number {rob}.";
        }

        if (paramsChanged)
            Params.OnParametersChanged();
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;
        if (!DA.GetData("Name", ref name) || name is null) return;

        RobotSystem? robotSystem = null;
        if (!DA.GetData("Robot system", ref robotSystem) || robotSystem is null) return;

        var initCommandsGH = new List<GH_Command>();
        DA.GetDataList("Init commands", initCommandsGH);
        var initCommands = initCommandsGH.Count > 0 ? new Robots.Commands.Group(initCommandsGH.Select(x => x.Value)) : null;

        var multiFileIndices = new List<int>();
        DA.GetDataList("Multi-file indices", multiFileIndices);

        double stepSize = 1;
        if (!DA.GetData("Step Size", ref stepSize)) return;

        if (robotSystem != _robotSystem)
        {
            _robotSystem = robotSystem;
            SetTargetInputs();
        }

        var toolpaths = new List<IToolpath>();

        foreach (var param in Params.Input.OfType<ToolpathParameter>())
        {
            var toolpathGh = new List<GH_Toolpath>();
            if (!DA.GetDataList(param.Name, toolpathGh))
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $"Input parameter {param.NickName} failed to collect data");
                DA.AbortComponentSolution();
                return;
            }

            var toolpath = new SimpleToolpath(toolpathGh.Where(t => t is not null).Select(t => t.Value));
            toolpaths.Add(toolpath);
        }

        var program = new Program(name, robotSystem, toolpaths, initCommands, multiFileIndices, stepSize);

        DA.SetData(0, program);

        if (program.Code is not null)
        {
            var path = DA.ParameterTargetPath(2);
            var structure = new GH_Structure<GH_String>();

            for (int i = 0; i < program.Code.Count; i++)
            {
                var tempPath = path.AppendElement(i);
                for (int j = 0; j < program.Code[i].Count; j++)
                {
                    structure.AppendRange(program.Code[i][j].Select(x => new GH_String(x)), tempPath.AppendElement(j));
                }
            }

            DA.SetDataTree(1, structure);
        }

        DA.SetData(2, program.Duration);

        if (program.Warnings.Count > 0)
        {
            DA.SetDataList(3, program.Warnings);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Warnings in program");
        }

        if (program.Errors.Count > 0)
        {
            DA.SetDataList(4, program.Errors);
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in program");
        }
    }
}
