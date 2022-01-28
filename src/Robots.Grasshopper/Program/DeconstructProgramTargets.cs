using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Data;

namespace Robots.Grasshopper;

public class DeconstructProgramTargets : GH_Component
{
    public DeconstructProgramTargets() : base("Deconstruct program targets", "DecProgTarg", "Exposes the calculated simulation data for all targets.", "Robots", "Utility") { }
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    public override Guid ComponentGuid => new("{B78BF8E5-D5F2-4DE6-8589-26E069BA3D5B}");
    protected override Bitmap Icon => Util.GetIcon("iconDeconstructProgramTarget");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddPlaneParameter("Planes", "P", "Program target planes", GH_ParamAccess.tree);
        pManager.AddNumberParameter("Joints", "J", "Program target joints", GH_ParamAccess.tree);
        pManager.AddTextParameter("Configuration", "C", "Program target configuration", GH_ParamAccess.tree);
        pManager.AddNumberParameter("Delta time", "T", "Program target time it takes to perform the motion", GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        IProgram? program = null;
        if (!DA.GetData(0, ref program) || program is null) return;

        if (program is not Program p)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, " Input program can't have custom code");
            DA.AbortComponentSolution();
            return;
        }

        var path = DA.ParameterTargetPath(0);
        var cellTargets = p.Targets;
        var groupCount = cellTargets[0].ProgramTargets.Count;

        var planes = new GH_Structure<GH_Plane>();
        var joints = new GH_Structure<GH_Number>();
        var configuration = new GH_Structure<GH_String>();
        var deltaTime = new GH_Structure<GH_Number>();

        for (int i = 0; i < groupCount; i++)
        {
            var tempPath = path.AppendElement(i);
            for (int j = 0; j < cellTargets.Count; j++)
            {
                planes.AppendRange(cellTargets[j].ProgramTargets[i].Kinematics.Planes.Select(x => new GH_Plane(x)), tempPath.AppendElement(j));
                joints.AppendRange(cellTargets[j].ProgramTargets[i].Kinematics.Joints.Select(x => new GH_Number(x)), tempPath.AppendElement(j));
                configuration.Append(new GH_String(cellTargets[j].ProgramTargets[i].Kinematics.Configuration.ToString()), tempPath);
                deltaTime.Append(new GH_Number(cellTargets[j].DeltaTime), tempPath);
            }
        }

        DA.SetDataTree(0, planes);
        DA.SetDataTree(1, joints);
        DA.SetDataTree(2, configuration);
        DA.SetDataTree(3, deltaTime);
    }
}
