namespace Robots.Commands;

public class Wait(double seconds, bool runBefore = false) : Command(runBefore: runBefore)
{
    public double Seconds { get; } = CheckNonNegative(seconds, nameof(seconds));

    public override string ToString() => $"Command (Wait {Seconds} secs)";
}
