using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;

namespace Robots.Grasshopper;

public class CreateProgram : GH_Component
{
    public CreateProgram() : base("Create program", "Program", "Creates a program, checks for possible issues and fixes common mistakes", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.quarternary;
    public override Guid ComponentGuid => new("{5186EFD5-C042-4CA9-A7D2-E143F4848DEF}");
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCreateProgram;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Program name", GH_ParamAccess.item, "DefaultProgram");
        pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system used in program", GH_ParamAccess.item);
        pManager.AddParameter(new ToolpathParameter(), "Targets 1", "T1", "List of targets or toolpaths for the first or only robot.", GH_ParamAccess.list);
        pManager.AddParameter(new ToolpathParameter(), "Targets 2", "T2", "List of targets or toolpaths for a second coordinated robot.", GH_ParamAccess.list);
        pManager.AddParameter(new CommandParameter(), "Init commands", "C", "Optional list of commands that will run at the start of the program", GH_ParamAccess.list);
        pManager.AddIntegerParameter("Multifile indices", "I", "Optional list of indices to split the program into multiple files. The indices correspond to the first target of the aditional files", GH_ParamAccess.list);
        pManager.AddNumberParameter("Step Size", "S", "Distance in mm to step through linear motions, used for error checking and program simulation. Smaller is more accurate but slower", GH_ParamAccess.item, 1);
        pManager[3].Optional = true;
        pManager[4].Optional = true;
        pManager[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        pManager.AddTextParameter("Code", "C", "Code", GH_ParamAccess.tree);
        pManager.AddNumberParameter("Duration", "D", "Program duration in seconds", GH_ParamAccess.item);
        pManager.AddTextParameter("Warnings", "W", "Warnings in program", GH_ParamAccess.list);
        pManager.AddTextParameter("Errors", "E", "Errors in program", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;
        GH_RobotSystem? robotSystem = null;
        var initCommandsGH = new List<GH_Command>();
        var toolpathsA = new List<GH_Toolpath>();
        var toolpathsB = new List<GH_Toolpath>();
        var multiFileIndices = new List<int>();
        double stepSize = 1;

        if (!DA.GetData(0, ref name) || name is null) { return; }
        if (!DA.GetData(1, ref robotSystem) || robotSystem is null) { return; }
        if (!DA.GetDataList(2, toolpathsA)) { return; }
        DA.GetDataList(3, toolpathsB);
        DA.GetDataList(4, initCommandsGH);
        DA.GetDataList(5, multiFileIndices);
        if (!DA.GetData(6, ref stepSize)) { return; }

        var initCommands = initCommandsGH.Count > 0 ? new Robots.Commands.Group(initCommandsGH.Select(x => x.Value)) : null;

        var toolpaths = new List<IToolpath>();

        var toolpathA = new SimpleToolpath(toolpathsA.Select(t => t.Value));
        toolpaths.Add(toolpathA);

        if (toolpathsB.Count > 0)
        {
            var toolpathB = new SimpleToolpath(toolpathsB.Select(t => t.Value));
            toolpaths.Add(toolpathB);
        }

        var program = new Program(name, robotSystem.Value, toolpaths, initCommands, multiFileIndices, stepSize);

        DA.SetData(0, new GH_Program(program));

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
