using GH_IO.Serialization;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Speed() : Goo<Speed, GH_Speed>("Speed", Speed.Default)
{
    const string DataKey = "Data";

    readonly record struct SpeedData(
        double TranslationSpeed,
        double RotationSpeed,
        double TranslationExternal,
        double RotationExternal,
        double TranslationAccel,
        double AxisAccel,
        double Time,
        string? Name)
    {
        internal static SpeedData From(Speed speed) =>
            new(
                speed.TranslationSpeed,
                speed.RotationSpeed,
                speed.TranslationExternal,
                speed.RotationExternal,
                speed.TranslationAccel,
                speed.AxisAccel,
                speed.Time,
                speed.HasName ? speed.Name : null);

        internal Speed ToSpeed() =>
            new(TranslationSpeed, RotationSpeed, TranslationExternal, RotationExternal, Name, TranslationAccel, AxisAccel, Time);
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
        return false;
    }

    public override bool Write(GH_IWriter writer)
    {
        if (Value is null)
            return false;

        writer.SetJson(DataKey, SpeedData.From(Value));
        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (!reader.TryGetJson<SpeedData>(DataKey, out var data))
            return false;

        Value = data.ToSpeed();
        return true;
    }

    protected override Speed DuplicateValue(Speed value) =>
        SpeedData.From(value).ToSpeed();
}
