using System.Diagnostics;
using System.Drawing;
using System.Windows.Forms;

namespace Robots.Grasshopper;

static class Util
{
    const string WikiUrl = "https://github.com/visose/Robots/wiki";

    public static Bitmap GetIcon(Type type) => GetIcon(type.Name);

    public static Bitmap GetIcon(string name)
    {
        var icon = $"Robots.Grasshopper.Resources.Icons.{name}.png";
        var assembly = typeof(RobotsInfo).Assembly;
        using var stream = assembly.GetManifestResourceStream(icon)
            ?? throw new FileNotFoundException($"Embedded icon '{icon}' was not found.");

        return new(stream);
    }

    public static string ComponentHelpUrl(IGH_InstanceDescription source) => HelpUrl(source, "Components");

    public static string ParameterHelpUrl(IGH_InstanceDescription source) => HelpUrl(source, "Parameters");

    public static void ReplaceHelpMenuItem(ToolStripDropDown menu, string url)
    {
        for (int i = menu.Items.Count - 1; i >= 0; i--)
        {
            if (menu.Items[i] is not ToolStripMenuItem item || !IsHelpMenuItem(item))
                continue;

            ToolStripMenuItem replacement = new(item.Text, item.Image, (_, _) => OpenUrl(url))
            {
                Enabled = item.Enabled,
                ShortcutKeyDisplayString = item.ShortcutKeyDisplayString,
                ShowShortcutKeys = item.ShowShortcutKeys,
                ToolTipText = url
            };

            menu.Items.RemoveAt(i);
            menu.Items.Insert(i, replacement);
            return;
        }

        throw new InvalidOperationException("Grasshopper help menu item was not found.");
    }

    static string HelpUrl(IGH_InstanceDescription source, string page) => $"{WikiUrl}/{page}#{WikiAnchor(source.Name)}";

    static bool IsHelpMenuItem(ToolStripMenuItem item)
    {
        if (string.IsNullOrEmpty(item.Text))
            return false;

        var text = item.Text
            .Replace("&", "", StringComparison.Ordinal)
            .Replace("…", "...", StringComparison.Ordinal)
            .Trim();

        return text.Equals("Help", StringComparison.Ordinal)
            || text.Equals("Help...", StringComparison.Ordinal);
    }

    static void OpenUrl(string url)
    {
        _ = Process.Start(new ProcessStartInfo(url) { UseShellExecute = true });
    }

    static string WikiAnchor(string name)
    {
        var chars = name
            .Trim()
            .ToLowerInvariant()
            .Select(static c => char.IsLetterOrDigit(c) ? c : '-')
            .ToArray();

        return CollapseHyphens(chars);
    }

    static string CollapseHyphens(char[] chars)
    {
        List<char> result = new(chars.Length);
        bool previousHyphen = true;

        foreach (char c in chars)
        {
            if (c == '-')
            {
                if (!previousHyphen)
                    result.Add(c);

                previousHyphen = true;
                continue;
            }

            result.Add(c);
            previousHyphen = false;
        }

        if (result.Count > 0 && result[^1] == '-')
            result.RemoveAt(result.Count - 1);

        return new([.. result]);
    }
}
