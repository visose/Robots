namespace Robots.Grasshopper;

static class ObsoleteComponent
{
    internal static void Fail(string replacement) =>
        throw new NotSupportedException($"This legacy component is no longer supported and will not run. Use Grasshopper's component upgrade command to replace it with the new {replacement} component, or place a new {replacement} component manually and reconnect the wires.");
}
