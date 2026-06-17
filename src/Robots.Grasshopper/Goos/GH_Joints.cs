using GH_IO.Serialization;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Joints() : Goo<double[], GH_Joints>("Joints", [])
{
    public override string ToString() => Value is null ? NullText : string.Join(",", Value.Select(x => $"{x:0.#####}"));

    protected override double[] Validate(double[] value) => Check(value);

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case double[] values:
                Value = Check(values);
                return true;
            case GH_Number number:
                Value = Check([number.Value]);
                return true;
            case GH_Integer integer:
                Value = Check([integer.Value]);
                return true;
            case GH_String text:
                {
                    if (string.IsNullOrWhiteSpace(text.Value))
                    {
                        Value = [];
                        return true;
                    }

                    string[] texts = text.Value.Split(',');
                    double[] values = new double[texts.Length];

                    for (int i = 0; i < texts.Length; i++)
                    {
                        if (!GH_Convert.ToDouble_Secondary(texts[i], ref values[i]))
                            return false;
                    }

                    Value = Check(values);
                    return true;
                }
        }

        return false;
    }

    public override bool Write(GH_IWriter writer)
    {
        if (Value is null)
            return false;

        writer.SetDoubleArray("Value", Value);
        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (!reader.ItemExists("Value"))
            return false;

        Value = Check(reader.GetDoubleArray("Value"));
        return true;
    }

    static double[] Check(double[] values)
    {
        if (Array.Exists(values, static value => !double.IsFinite(value)))
            throw new ArgumentException("Joint values must be finite.", nameof(values));

        return [.. values];
    }

    protected override double[] DuplicateValue(double[] value) => [.. value];
}
