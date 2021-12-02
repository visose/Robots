using Rhino.Geometry;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class GH_Frame : GH_Goo<Frame>
    {
        public GH_Frame() { Value = Frame.Default; }
        public GH_Frame(GH_Frame goo) { Value = goo.Value; }
        public GH_Frame(Frame native) { Value = native; }
        public override IGH_Goo Duplicate() => new GH_Frame(this);
        public override bool IsValid => true;
        public override string TypeName => "Frame";
        public override string TypeDescription => "Frame";
        public override string ToString() => Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Frame frame:
                    Value = frame;
                    return true;
                case GH_Plane plane:
                    Value = new Frame(plane.Value);
                    return true;
                case GH_Point point:
                    Value = new Frame(new Plane(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                    return true;
                default:
                    return false;
            }
        }
    }
}
