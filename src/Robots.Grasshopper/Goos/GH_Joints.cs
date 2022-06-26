using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Joints : GH_Goo<double[]>
{
    public GH_Joints() { Value = Array.Empty<double>(); }
    public GH_Joints(GH_Joints goo) { Value = goo.Value; }
    public GH_Joints(double[] native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_Joints(this);
    public override bool IsValid => true;
    public override string TypeName => "Joints";
    public override string TypeDescription => "Joints";
    public override string ToString() => string.Join(",", Value.Select(x => $"{x:0.#####}"));

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case GH_Number number:
                Value = new[] { number.Value };
                return true;
            case GH_String text:
                {
                    if (string.IsNullOrWhiteSpace(text.Value))
                    {
                        Value = new double[0];
                        return true;
                    }

                    string[] texts = text.Value.Split(',');
                    double[] values = new double[texts.Length];

                    for (int i = 0; i < texts.Length; i++)
                    {
                        if (!GH_Convert.ToDouble_Secondary(texts[i], ref values[i]))
                            return false;
                    }

                    Value = values;
                    return true;
                }
        }

        return false;
    }

    public override bool CastTo<Q>(ref Q target)
    {
        if (typeof(Q).IsAssignableFrom(typeof(double[])))
        {
            target = (Q)(object)Value;
            return true;
        }

        return false;
    }
}
