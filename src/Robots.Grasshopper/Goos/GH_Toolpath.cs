
namespace Robots.Grasshopper;

public class GH_Toolpath() : Goo<IToolpath, GH_Toolpath>("Toolpath", new SimpleToolpath())
{
    public override string ToString()
    {
        return Value?.Targets switch
        {
            IList<Target> targets => $"Toolpath with ({targets.Count} targets)",
            Target target => target.ToString() ?? "Target",
            null => "Null Toolpath",
            _ => "Toolpath",
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
