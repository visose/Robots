using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Zone : GH_Goo<Zone>
{
    public GH_Zone() { Value = Zone.Default; }
    public GH_Zone(GH_Zone goo) { Value = goo.Value; }
    public GH_Zone(Zone native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_Zone(this);
    public override bool IsValid => true;
    public override string TypeName => "Zone";
    public override string TypeDescription => "Zone";
    public override string ToString() => Value.ToString();
    public override object ScriptVariable() => Value;

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case Zone zone:
                Value = zone;
                return true;
            case GH_Number number:
                Value = new Zone(number.Value);
                return true;
            case GH_String text:
                {
                    string[] texts = text.Value.Split(',');
                    double[] values = new double[texts.Length];

                    for (int i = 0; i < texts.Length; i++)
                        if (!GH_Convert.ToDouble_Secondary(texts[i], ref values[i]))
                            return false;

                    if (texts.Length == 1)
                    {
                        Value = new Zone(values[0]);
                        return true;
                    }
                    else if (texts.Length == 2)
                    {
                        Value = new Zone(values[0], values[1]);
                        return true;
                    }

                    break;
                }
        }

        double value = 0;
        if (GH_Convert.ToDouble_Secondary(source, ref value))
        {
            Value = new Zone(value);
            return true;
        }
        else
        {
            return false;
        }
    }
}
