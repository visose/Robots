using System.Xml;
using System.Xml.Linq;
using Rhino.Geometry;

namespace Robots;

static class XmlExtensions
{
    extension(XElement element)
    {
        public XElement GetElement(string name) =>
            element.Element(XName.Get(name))
                ?? throw new ArgumentException($"XML tag '{element.Name}' is missing the '{name}' element.");

        public XElement? GetElementOrDefault(string name) => element.Element(XName.Get(name));

        public string? GetStringOrDefault(string name) => element.Attribute(name)?.Value;

        public string GetString(string name) =>
            element.GetStringOrDefault(name)
                ?? throw new ArgumentException($"XML tag '{element.Name}' is missing the '{name}' attribute.");

        public bool GetBoolOrDefault(string name)
        {
            string? value = element.GetStringOrDefault(name);
            return value is not null && XmlConvert.ToBoolean(value);
        }

        public bool AttributeExists(string name) => element.Attribute(XName.Get(name)) is not null;

        public double GetDoubleAttribute(string name) => XmlConvert.ToDouble(element.GetString(name));

        public double? GetDoubleOrNull(string name)
        {
            string? value = element.GetStringOrDefault(name);
            return value is null ? null : XmlConvert.ToDouble(value);
        }

        public int GetIntAttribute(string name) => XmlConvert.ToInt32(element.GetString(name));

        public int? GetIntOrNull(string name)
        {
            string? value = element.GetStringOrDefault(name);
            return value is null ? null : XmlConvert.ToInt32(value);
        }

        public Plane ToPlane() => GeometryUtil.QuaternionToPlane(
            element.GetDoubleAttribute("x"),
            element.GetDoubleAttribute("y"),
            element.GetDoubleAttribute("z"),
            element.GetDoubleAttribute("q1"),
            element.GetDoubleAttribute("q2"),
            element.GetDoubleAttribute("q3"),
            element.GetDoubleAttribute("q4"));

        public Point3d? ToPointOrNull()
        {
            if (!element.AttributeExists("x")
                && !element.AttributeExists("y")
                && !element.AttributeExists("z"))
            {
                return null;
            }

            return new(
                element.GetDoubleAttribute("x"),
                element.GetDoubleAttribute("y"),
                element.GetDoubleAttribute("z"));
        }
    }
}
