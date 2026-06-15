namespace Robots.Grasshopper;

public class DegreesToRadians() : Component(
    "Degrees To Radians",
    "Converts manufacturer-specific joint degrees to radians.",
    "Utility",
    "{C10B3A17-5C19-4805-ACCF-839B85C4D21C}")
{
    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddNumberParameter("Degrees", "D", "Joint values in manufacturer-specific degrees.", GH_ParamAccess.list);
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Robot system used for the conversion.", GH_ParamAccess.item);
        _ = pManager.AddIntegerParameter("Mechanical Group", "G", "Mechanical group index.", GH_ParamAccess.item, 0);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new JointsParameter(), "Radians", "R", "Joint values in radians.", GH_ParamAccess.item);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var degrees = DA.List<double>(0);
        var robotSystem = DA.Get<RobotSystem>(1);
        var group = DA.Get<int>(2);
        var radians = new double[degrees.Length];

        for (int i = 0; i < degrees.Length; i++)
            radians[i] = robotSystem.DegreeToRadian(degrees[i], i, group);

        _ = DA.SetData(0, radians);
    }
}
