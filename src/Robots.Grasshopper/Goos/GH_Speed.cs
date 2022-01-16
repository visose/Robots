using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Speed : GH_Goo<Speed>
{
    public GH_Speed() { Value = Speed.Default; }
    public GH_Speed(GH_Speed goo) { Value = goo.Value; }
    public GH_Speed(Speed native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_Speed(this);
    public override bool IsValid => true;
    public override string TypeName => "Speed";
    public override string TypeDescription => "Speed";
    public override string ToString() => Value.ToString();

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case Speed speed:
                Value = speed;
                return true;
            case GH_Number number:
                Value = new Speed(number.Value);
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
                        Value = new Speed(values[0]);
                        return true;
                    }
                    else if (texts.Length == 2)
                    {
                        Value = new Speed(values[0], values[1]);
                        return true;
                    }

                    break;
                }
        }
        return false;
    }
}
