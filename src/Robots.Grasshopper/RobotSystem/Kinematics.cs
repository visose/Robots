
namespace Robots.Grasshopper;

public class Kinematics() : Component(
    "Kinematics",
    "Solves robot kinematics for one target per robot.",
    "Components",
    "{EFDA05EB-B281-4703-9C9E-B5F98A9B2E1D}",
    GH_Exposure.quinary)
{
    readonly Dictionary<(RobotSystem RobotSystem, int Iteration), List<KinematicSolution>> _prevKinematics = [];

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system used for the kinematics solution.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new TargetParameter(), "Target", "T", "One target per robot.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new JointsParameter(), "Previous Joints", "J", "Optional previous joint values. If the pose is ambiguous, these values are used to select the closest solution.", GH_ParamAccess.list);
        _ = pManager.AddBooleanParameter("Display Geometry", "M", "Output posed robot meshes.", GH_ParamAccess.item, false);
        pManager[2].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddMeshParameter("Meshes", "M", "Posed robot system meshes.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new JointsParameter(), "Joints", "J", "Solved joint rotations in radians.", GH_ParamAccess.item);
        _ = pManager.AddPlaneParameter("Planes", "P", "Solved joint planes.", GH_ParamAccess.list);
        _ = pManager.AddTextParameter("Errors", "E", "Kinematic solution errors.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var robotSystem = DA.Get<RobotSystem>(0);
        var targets = DA.List<Target>(1);
        var previousJoints = DA.MaybeList<double[]>(2);
        var drawMeshes = DA.Get<bool>(3);

        double[][]? prevJoints = null;
        var key = (robotSystem, DA.Iteration);
        _ = _prevKinematics.TryGetValue(key, out var prevKinematics);

        if (previousJoints.Length > 0)
        {
            prevJoints = previousJoints;
        }
        else if (prevKinematics is not null)
        {
            prevJoints = [.. prevKinematics.Select(solution => solution.Joints)];
        }

        var kinematics = robotSystem.Kinematics(targets, prevJoints);
        var errors = kinematics.AllErrors();

        if (errors.Length > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Solution has errors.");

        if (errors.Contains("Target out of reach."))
        {
            if (prevKinematics is not null)
                kinematics = prevKinematics;
        }
        else
        {
            _prevKinematics[key] = kinematics;
        }

        if (drawMeshes)
            _ = DA.SetDataList(0, RhinoMeshPoser.Pose(robotSystem, kinematics, targets));

        _ = DA.SetData(1, kinematics.AllJoints());
        _ = DA.SetDataList(2, kinematics.AllPlanes());
        _ = DA.SetDataList(3, errors);
    }
}
