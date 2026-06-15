using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class DrawSimpleTrail() : Component(
    "Simple Trail",
    "Draws a trail behind the simulated TCP.",
    "Utility",
    "{20F09C83-25A5-453B-B0C9-673CD784A52F}",
    GH_Exposure.secondary)
{
    SimpleTrail? _trail;
    Program? _program;
    int _mechanicalGroup;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program output from the simulation component.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Length", "L", "Trail length in model units.", GH_ParamAccess.item, 500);
        _ = pManager.AddIntegerParameter("Mechanical Group", "M", "Mechanical group index.", GH_ParamAccess.item, 0);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddCurveParameter("Trail", "C", "TCP trail curve.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var inputProgram = DA.Get<IProgram>(0);
        var length = DA.Get<double>(1);
        var mechanicalGroup = DA.Get<int>(2);

        if (inputProgram is not Program program || !program.HasSimulation)
            throw new RuntimeWarningException("Input program cannot be animated.");

        if (!ReferenceEquals(program, _program) || mechanicalGroup != _mechanicalGroup)
        {
            _program = program;
            _mechanicalGroup = mechanicalGroup;
            _trail = new(program, length, mechanicalGroup);
        }

        if (_trail is not null)
        {
            _trail.Length = length;
            _trail.Update();

            if (_trail.Polyline.Count >= 2)
                _ = DA.SetData(0, new GH_Curve(_trail.Polyline.ToNurbsCurve()));
        }
    }
}
