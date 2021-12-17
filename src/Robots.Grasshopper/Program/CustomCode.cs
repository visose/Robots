using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;

namespace Robots.Grasshopper;

public class CustomCode : GH_Component
{
    public CustomCode() : base("Custom code", "Custom", "Creates a program using manufacturer specific custom code. This program cannot be simulated", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.quarternary;
    public override Guid ComponentGuid => new("{FF997511-4A84-4426-AB62-AF94D19FF58F}");
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCustomCode;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        pManager.AddTextParameter("Code", "C", "Custom code", GH_ParamAccess.tree);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        GH_Program? program = null;

        if (!DA.GetData(0, ref program) || program is null) { return; }
        if (!DA.GetDataTree(1, out GH_Structure<GH_String> codeTree)) { return; }

        if (program?.Value is not Program p)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " Input program can't have custom code.");
            return;
        }

        var code = new List<List<List<string>>>
            {
                new List<List<string>>()
            };

        foreach (var branch in codeTree.Branches)
        {
            code[0].Add(branch.Select(s => s.Value).ToList());
        }

        var programCode = program.Value.Code;
        if (programCode is not null && programCode.Count > 0)
        {
            //var copyCode = programCode.ToList();

            //for (int i = 0; i < copyCode.Count; i++)
            //{
            //    copyCode[i] = copyCode[i].ToList();

            //    for (int j = 0; j < copyCode[i].Count; j++)
            //        copyCode[i][j] = copyCode[i][j].ToList();
            //}

            //copyCode[0][0] = code;

            var newProgram = p.CustomCode(code);
            DA.SetData(0, new GH_Program(newProgram));
        }
    }
}
