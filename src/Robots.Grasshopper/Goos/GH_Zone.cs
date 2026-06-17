using GH_IO.Serialization;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Zone() : Goo<Zone, GH_Zone>("Zone", Zone.Default)
{
    const string DataKey = "Data";

    readonly record struct ZoneData(double Distance, double Rotation, double RotationExternal, string? Name)
    {
        internal static ZoneData From(Zone zone) =>
            new(zone.Distance, zone.Rotation, zone.RotationExternal, zone.HasName ? zone.Name : null);

        internal Zone ToZone() =>
            new(Distance, Rotation, RotationExternal, Name);
    }

    public override bool CastFrom(object source)
    {
        if (base.CastFrom(source))
            return true;

        switch (source)
        {
            case GH_Number number:
                Value = new(number.Value);
                return true;
            case GH_String text:
                {
                    string[] texts = text.Value.Split(',');
                    double[] values = new double[texts.Length];

                    for (int i = 0; i < texts.Length; i++)
                    {
                        if (!GH_Convert.ToDouble_Secondary(texts[i], ref values[i]))
                            return false;
                    }

                    if (texts.Length == 1)
                    {
                        Value = new(values[0]);
                        return true;
                    }
                    else if (texts.Length == 2)
                    {
                        Value = new(values[0], values[1]);
                        return true;
                    }

                    break;
                }
        }

        double value = 0;
        if (GH_Convert.ToDouble_Secondary(source, ref value))
        {
            Value = new(value);
            return true;
        }
        else
        {
            return false;
        }
    }

    public override bool Write(GH_IWriter writer)
    {
        if (Value is null)
            return false;

        writer.SetJson(DataKey, ZoneData.From(Value));
        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (!reader.TryGetJson<ZoneData>(DataKey, out var data))
            return false;

        Value = data.ToZone();
        return true;
    }

    protected override Zone DuplicateValue(Zone value) =>
        ZoneData.From(value).ToZone();
}
