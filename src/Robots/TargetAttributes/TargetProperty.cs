namespace Robots;

public abstract class TargetProperty
{
    protected string? _name;

    protected TargetProperty(string? name)
    {
        if (name is not null)
            Name = name;
    }

    /// <summary>
    /// Name of the attribute
    /// </summary>
    public string Name
    {
        get => _name.NotNull();
        private set
        {
            if (!Program.IsValidIdentifier(value, out var error))
                throw new ArgumentException($"{GetType().Name} {error}", nameof(value));

            _name = value;
        }
    }

    public bool HasName => _name is not null;

    public T CloneWithName<T>(string name) where T : TargetProperty
    {
        var attribute = (T)MemberwiseClone();
        attribute.Name = name;
        return attribute;
    }

    protected static double CheckFinite(double value, string name)
    {
        return Util.CheckFinite(value, name);
    }

    protected static double CheckNonNegative(double value, string name)
    {
        _ = CheckFinite(value, name);
        ArgumentOutOfRangeException.ThrowIfNegative(value, name);
        return value;
    }

    protected static double CheckPositive(double value, string name)
    {
        _ = CheckFinite(value, name);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, name);
        return value;
    }
}
