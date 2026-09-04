#pragma warning disable CA1716 // Stop is intentional in the Commands namespace.

namespace Robots.Commands;

public class Stop(bool runBefore = false) : Command(runBefore: runBefore)
{
    public override string ToString() => "Command (Stop)";
}
