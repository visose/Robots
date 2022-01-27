using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class DrawSimpleTrail : GH_Component
{
    SimpleTrail? _trail;
    Program? _program;

    public DrawSimpleTrail() : base("Simple trail", "Trail", "Draws a trail behind the TCP. To be used with the simulation component.", "Robots", "Utility") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{20F09C83-25A5-453B-B0C9-673CD784A52F}");
    protected override Bitmap Icon => Util.GetIcon("iconSimpleTrail");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Connect to the program output from the simulation component", GH_ParamAccess.item);
        pManager.AddNumberParameter("Length", "L", "Length of trail", GH_ParamAccess.item, 500);
        pManager.AddIntegerParameter("Mechanical group", "M", "Index of mechanical group", GH_ParamAccess.item, 0);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddCurveParameter("Trail", "C", "Trail polyline", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        GH_Program? ghProgram = null;
        double length = 0;
        int mechanicalGroup = 0;

        if (!DA.GetData(0, ref ghProgram) || ghProgram is null) { return; }
        if (!DA.GetData(1, ref length)) { return; }
        if (!DA.GetData(2, ref mechanicalGroup)) { return; }

        if (ghProgram?.Value is not Program p)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " Input program can't have custom code.");
            return;
        }

        if (ghProgram.Value != _program)
        {
            _program = p;
            _trail = new SimpleTrail(_program, length, mechanicalGroup);
        }

        if (_trail is not null)
        {
            _trail.Length = length;
            _trail.Update();

            if (_trail.Polyline.Count >= 2)
                DA.SetData(0, new GH_Curve(_trail.Polyline.ToNurbsCurve()));
        }
    }
}
