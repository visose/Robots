namespace Robots;

public abstract class TargetAttribute
{
    /// <summary>
    /// Name of the attribute
    /// </summary>
    public string? Name { get; internal set; }

    public T CloneWithName<T>(string name) where T : TargetAttribute
    {
        var attribute = (T)MemberwiseClone();
        attribute.Name = name;
        return attribute;
    }
}
