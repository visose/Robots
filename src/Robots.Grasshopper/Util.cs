using System.Drawing;

namespace Robots.Grasshopper;

static class Util
{
    public static Bitmap GetIcon(Type type) => GetIcon(type.Name);

    public static Bitmap GetIcon(string name)
    {
        var icon = $"Robots.Grasshopper.Resources.Icons.{name}.png";
        var assembly = typeof(RobotsInfo).Assembly;
        using var stream = assembly.GetManifestResourceStream(icon)
            ?? throw new FileNotFoundException($"Embedded icon '{icon}' was not found.");

        return new(stream);
    }
}
