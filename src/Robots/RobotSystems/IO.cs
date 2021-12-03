using System.Xml.Linq;

namespace Robots;

public class IO
{
    public string[] DO { get; }
    public string[] DI { get; }
    public string[] AO { get; }
    public string[] AI { get; }

    public IO(XElement? element)
    {
        DO = GetNames(element, "DO");
        DI = GetNames(element, "DI");
        AO = GetNames(element, "AO");
        AI = GetNames(element, "AI");
    }

    string[] GetNames(XElement? ioElement, string element)
    {
        var e = ioElement?.GetElementOrDefault(element);

        if (e is null)
            return new string[0];

        return e.GetAttribute("names").Split(',');
    }
}