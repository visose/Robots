using System.Drawing;

namespace Robots.Grasshopper;

static class Util
{
    public static Bitmap GetIcon(string name)
    {
        var icon = $"Robots.Grasshopper.Assets.Embed.{name}.png";
        var assembly = typeof(RobotsInfo).Assembly;
        using var stream = assembly.GetManifestResourceStream(icon);
        return new Bitmap(stream);
    }
}