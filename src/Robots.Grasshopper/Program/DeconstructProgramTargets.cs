using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class DeconstructProgramTargets() : Component(
    "Deconstruct Program Targets",
    "Exposes the calculated simulation data for all targets.",
    "Utility",
    "{B78BF8E5-D5F2-4DE6-8589-26E069BA3D5B}",
    GH_Exposure.secondary)
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Robot program with simulation data.", GH_ParamAccess.item);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddPlaneParameter("Planes", "P", "Calculated target planes.", GH_ParamAccess.tree);
        _ = pManager.AddNumberParameter("Joints", "J", "Calculated target joints in radians.", GH_ParamAccess.tree);
        _ = pManager.AddTextParameter("Configuration", "C", "Calculated target configuration.", GH_ParamAccess.tree);
        _ = pManager.AddNumberParameter("Delta Time", "T", "Motion time for each target in seconds.", GH_ParamAccess.tree);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        if (DA.Get<IProgram>(0) is not Program p)
            throw new ArgumentException("Input program does not expose simulation targets.");

        var path = DA.ParameterTargetPath(0);
        var systemTargets = p.Targets;

        if (systemTargets.Count == 0)
            throw new InvalidOperationException("Program has no calculated targets.");

        var groupCount = systemTargets[0].ProgramTargets.Count;

        var planes = DataAccess.PlaneTree();
        var joints = DataAccess.NumberTree();
        var configuration = DataAccess.TextTree();
        var deltaTime = DataAccess.NumberTree();

        for (int i = 0; i < groupCount; i++)
        {
            var tempPath = path.AppendElement(i);
            for (int j = 0; j < systemTargets.Count; j++)
            {
                var systemTarget = systemTargets[j];
                var solution = systemTarget.ProgramTargets[i].Kinematics;
                var targetPath = tempPath.AppendElement(j);

                planes.AppendPlanes(solution.Planes, targetPath);
                joints.AppendNumbers(solution.Joints, targetPath);
                configuration.AppendText(solution.Configuration.ToString(), tempPath);
                deltaTime.AppendNumber(systemTarget.DeltaTime, tempPath);
            }
        }

        _ = DA.SetDataTree(0, planes);
        _ = DA.SetDataTree(1, joints);
        _ = DA.SetDataTree(2, configuration);
        _ = DA.SetDataTree(3, deltaTime);
    }
}
