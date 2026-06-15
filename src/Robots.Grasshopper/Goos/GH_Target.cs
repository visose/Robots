using Rhino.Geometry;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Target() : Goo<Target, GH_Target>("Target", Target.Default)
{
    public override bool CastFrom(object source)
    {
        if (base.CastFrom(source))
            return true;

        switch (source)
        {
            case GH_Point point:
                Value = new CartesianTarget(new(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                return true;
            case GH_Plane plane:
                Value = new CartesianTarget(plane.Value);
                return true;
            case GH_String text:
                {
                    string[] jointsText = text.Value.Split(',');

                    if (jointsText.Length is not 6 and not 7)
                        return false;

                    var joints = new double[jointsText.Length];

                    for (int i = 0; i < jointsText.Length; i++)
                        if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref joints[i])) return false;

                    Value = new JointTarget(joints);
                    return true;
                }
        }

        return false;
    }
}
