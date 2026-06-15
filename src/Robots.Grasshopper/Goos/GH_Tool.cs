using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Tool() : Goo<Tool, GH_Tool>("Tool", Tool.Default)
{
    public override bool CastTo<TGoo>(ref TGoo target)
    {
        if (base.CastTo(ref target))
            return true;

        if (Value is not null && typeof(TGoo) == typeof(GH_Plane))
        {
            target = (TGoo)(object)new GH_Plane(Value.Tcp);
            return true;
        }

        return false;
    }
}
