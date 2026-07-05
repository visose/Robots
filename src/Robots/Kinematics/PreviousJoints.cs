namespace Robots;

readonly ref struct PreviousJoints
{
    public PreviousJoints(double[]? values)
    {
        Values = values.AsSpan();
        HasValue = values is not null;
    }

    public PreviousJoints(ReadOnlySpan<double> values)
    {
        Values = values;
        HasValue = true;
    }

    public bool HasValue { get; }
    public ReadOnlySpan<double> Values { get; }
    public int Length => Values.Length;
    public double this[int index] => Values[index];
}
