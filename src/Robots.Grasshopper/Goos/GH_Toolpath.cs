
namespace Robots.Grasshopper;

public class GH_Toolpath() : Goo<IToolpath, GH_Toolpath>("Toolpath", new SimpleToolpath())
{
    public override string ToString()
    {
        return Value switch
        {
            Target target => target.ToString() ?? "Target",
            IToolpath toolpath => $"Toolpath with ({toolpath.Targets.Count} targets)",
            null => "Null Toolpath",
        };
    }

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case GH_Target target:
                Value = target.Value;
                return true;
            default:
                return base.CastFrom(source);
        }
    }
}
