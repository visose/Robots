#pragma warning disable CA1716 // Stop is intentional in the Commands namespace.

namespace Robots.Grasshopper.Commands;

public class Stop() : CommandComponent(
    "Stop Program",
    "Pauses the program until an operator resumes it.",
    "{80E4E1AD-D1C0-441F-BDC5-5E810BCECE61}",
    GH_Exposure.secondary)
{
    protected override Command SolveCommand(IGH_DataAccess DA) => new Robots.Commands.Stop();
}
