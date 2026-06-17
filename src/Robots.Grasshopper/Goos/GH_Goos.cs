using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public abstract class Goo<T, TGoo> : GH_Goo<T>
    where TGoo : Goo<T, TGoo>, new()
{
    readonly string _typeName;

    protected Goo(string typeName, T? initialValue = default)
    {
        _typeName = typeName;

        if (initialValue is not null)
            SetValue(initialValue);
    }

    public override bool IsValid => Value is not null;
    public override string TypeName => _typeName;
    public override string TypeDescription => _typeName;

    protected string NullText => $"Null {TypeName}";

    public override IGH_Goo Duplicate()
    {
        var goo = new TGoo();

        if (Value is null)
        {
            goo.Value = default!;
        }
        else
        {
            goo.SetValue(DuplicateValue(Value));
        }

        return goo;
    }
    public override string ToString() => Value?.ToString() ?? NullText;

    public override bool CastFrom(object source)
    {
        if (source is not T value)
            return false;

        SetValue(value);
        return true;
    }

    public override bool CastTo<TCast>(ref TCast target)
    {
        if (Value is null || !typeof(TCast).IsAssignableFrom(typeof(T)))
            return false;

        target = (TCast)(object)Value;
        return true;
    }

    internal void SetValue(T value) => Value = Validate(value);

    protected virtual T Validate(T value) => value;

    protected virtual T DuplicateValue(T value) => value;
}

public sealed class GH_Command() : Goo<Command, GH_Command>("Command", Command.Default)
{
    public GH_Command(Command value) : this() { SetValue(value); }
}

public sealed class GH_PostProcessor() : Goo<IPostProcessor, GH_PostProcessor>("Post Processor")
{
    public override string ToString() => "Post Processor" + (Value is null ? "" : $" ({Value.GetType().Name})");
}

public sealed class GH_Program() : Goo<IProgram, GH_Program>("Program");
