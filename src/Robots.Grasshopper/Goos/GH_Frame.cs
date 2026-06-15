using Rhino.Geometry;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Frame() : Goo<Frame, GH_Frame>("Frame", Frame.Default)
{
    public override bool CastFrom(object source)
    {
        if (base.CastFrom(source))
            return true;

        switch (source)
        {
            case GH_Plane plane:
                Value = new(plane.Value);
                return true;
            case GH_Point point:
                Value = new(new(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                return true;
            default:
                return false;
        }
    }

    public override bool CastTo<TGoo>(ref TGoo target)
    {
        if (base.CastTo(ref target))
            return true;

        if (Value is not null && typeof(TGoo) == typeof(GH_Plane))
        {
            target = (TGoo)(object)new GH_Plane(Value.Plane);
            return true;
        }

        return false;
    }
}
