using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_PostProcessor : GH_Goo<IPostProcessor>
{
    public GH_PostProcessor() { Value = null!; }
    public GH_PostProcessor(IPostProcessor native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_PostProcessor(Value);
    public override bool IsValid => true;
    public override string TypeName => "PostProcessor";
    public override string TypeDescription => "PostProcessor";
    public override string ToString() => "PostProcessor" + (Value is null ? "" : $"({Value.GetType().Name})");

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case IPostProcessor post:
                Value = post;
                return true;
            default:
                return false;
        }
    }

    public override bool CastTo<Q>(ref Q target)
    {
        if (typeof(Q).IsAssignableFrom(typeof(IPostProcessor)))
        {
            target = (Q)(object)Value;
            return true;
        }

        return false;
    }
}
