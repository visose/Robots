using System.Collections.Generic;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class GH_Toolpath : GH_Goo<IToolpath>
    {
        public GH_Toolpath() { Value = new SimpleToolpath(); }
        public GH_Toolpath(IToolpath native) { Value = native; }
        public override IGH_Goo Duplicate() => new GH_Toolpath(Value);
        public override bool IsValid => true;
        public override string TypeName => "Toolpath";
        public override string TypeDescription => "Toolpath";
        public override string ToString()
        {
            switch (Value.Targets)
            {
                case IList<Target> _:
                    var targets = Value.Targets as IList<Target>;
                    return $"Toolpath with ({targets.Count} targets)";
                case Target target:
                    return target.ToString();
                default:
                    return "Toolpath";
            }
        }
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case GH_Target target:
                    Value = target?.Value;
                    return true;
                case IToolpath toolpath:
                    Value = toolpath;
                    return true;
            }
            return false;
        }
    }
}
