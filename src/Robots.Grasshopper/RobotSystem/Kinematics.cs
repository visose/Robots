using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class Kinematics : GH_Component
{
    public Kinematics() : base("Kinematics", "K", "Inverse and forward kinematics for a single target, or list of targets when using a robot cell with coordinated robots.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.quinary;
    public override Guid ComponentGuid => new("{EFDA05EB-B281-4703-9C9E-B5F98A9B2E1D}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconKinematics");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
        pManager.AddParameter(new TargetParameter(), "Target", "T", "One target per robot", GH_ParamAccess.list);
        pManager.AddParameter(new JointsParameter(), "Previous joints", "J", "Optional previous joint values. If the pose is ambiguous is will select one based on this previous position.", GH_ParamAccess.list);
        pManager.AddBooleanParameter("Display geometry", "M", "Display mesh geometry of the robot", GH_ParamAccess.item, false);
        pManager[2].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddMeshParameter("Meshes", "M", "Robot system's meshes", GH_ParamAccess.list);
        pManager.AddParameter(new JointsParameter(), "Joints", "J", "Robot system's joint rotations.", GH_ParamAccess.item);
        pManager.AddPlaneParameter("Planes", "P", "Robot system's joint lanes", GH_ParamAccess.list);
        pManager.AddTextParameter("Errors", "E", "Errors in kinematic solution", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        RobotSystem? robotSystem = null;
        var ghTargets = new List<GH_Target>();
        var ghPrevJoints = new List<GH_Joints>();
        bool drawMeshes = false;

        if (!DA.GetData(0, ref robotSystem) || robotSystem is null) return;
        if (!DA.GetDataList(1, ghTargets)) return;
        DA.GetDataList(2, ghPrevJoints);
        if (!DA.GetData(3, ref drawMeshes)) return;

        var prevJoints = ghPrevJoints.Count == 0
            ? null
            : ghPrevJoints.Select(v => v.Value).ToList();

        var targets = ghTargets.Select(x => x.Value).ToList();
        var kinematics = robotSystem.Kinematics(targets, prevJoints);
        var errors = kinematics.SelectMany(x => x.Errors);

        if (errors.Any())
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");
        }

        var joints = kinematics.SelectMany(x => x.Joints).ToArray();
        var planes = kinematics.SelectMany(x => x.Planes);

        if (drawMeshes)
        {
            var meshes = RhinoMeshPoser.Pose(robotSystem, kinematics, targets);
            DA.SetDataList(0, meshes.Select(x => new GH_Mesh(x)));
        }

        DA.SetData(1, joints);
        DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
        DA.SetDataList(3, errors);
    }
}