using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Tool : GH_Goo<Tool>
{
    public GH_Tool() { Value = Tool.Default; }
    public GH_Tool(GH_Tool goo) { Value = goo.Value; }
    public GH_Tool(Tool native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_Tool(this);
    public override bool IsValid => true;
    public override string TypeName => "Tool";
    public override string TypeDescription => "Tool";
    public override string ToString() => Value.ToString();
    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case Tool tool:
                Value = tool;
                return true;
            default:
                return false;
        }
    }
}
