using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class GH_Command : GH_Goo<Command>
    {
        public GH_Command() { Value = Command.Default; }
        public GH_Command(GH_Command goo) { Value = goo.Value; }
        public GH_Command(Command native) { Value = native; }
        public override IGH_Goo Duplicate() => new GH_Command(this);
        public override bool IsValid => true;
        public override string TypeName => "Command";
        public override string TypeDescription => "Command";
        public override string ToString() => Value?.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Command command:
                    Value = command;
                    return true;
                default:
                    return false;
            }
        }
    }
}
