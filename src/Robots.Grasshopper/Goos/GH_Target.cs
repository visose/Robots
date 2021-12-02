using Rhino.Geometry;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper
{
    public class GH_Target : GH_Goo<Target>
    {
        public GH_Target() { Value = Target.Default; }
        public GH_Target(GH_Target goo) { Value = goo.Value; }
        public GH_Target(Target native) { Value = native; }
        public override IGH_Goo Duplicate() => new GH_Target(this);
        public override bool IsValid => true;
        public override string TypeName => "Target";
        public override string TypeDescription => "Target";
        public override string ToString() => Value.ToString();
        public override object ScriptVariable() => Value;

        public override bool CastFrom(object source)
        {
            switch (source)
            {
                case Target target:
                    Value = target;
                    return true;
                case GH_Point point:
                    Value = new CartesianTarget(new Plane(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                    return true;
                case GH_Plane plane:
                    Value = new CartesianTarget(plane.Value);
                    return true;
                case GH_String text:
                    {
                        string[] jointsText = text.Value.Split(',');
                        if (jointsText.Length != 6) return false;

                        var joints = new double[6];
                        for (int i = 0; i < 6; i++)
                            if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref joints[i])) return false;

                        Value = new JointTarget(joints);
                        return true;
                    }
            }

            return false;
        }
    }
}
