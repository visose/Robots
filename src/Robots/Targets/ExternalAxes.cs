using static Robots.Util;

namespace Robots;

public static class ExternalAxes
{
    public const string AbbUnspecified = "9E9";

    public static double[] Values(params double[] values)
    {
        return CheckFinite(values, nameof(values), "External axis values must be finite.");
    }

    public static string[] Custom(params string?[] values)
    {
        return [.. values.Select(value => value ?? "")];
    }

    public static string[] AbbUnspecifiedAxes(int count)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(count);
        return [.. Enumerable.Repeat(AbbUnspecified, count)];
    }
}
