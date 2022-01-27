namespace Robots;

public abstract class TargetAttribute
{
    string? _name;

    protected TargetAttribute(string? name)
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
                throw new ArgumentException($" {GetType().Name} {error}", nameof(value));

            _name = value;
        }
    }

    public bool HasName => _name is not null;

    public T CloneWithName<T>(string name) where T : TargetAttribute
    {
        var attribute = (T)MemberwiseClone();
        attribute.Name = name;
        return attribute;
    }
}