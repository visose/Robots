using System.Drawing;

namespace Robots.Grasshopper;

public abstract class Param<T, TGoo>(
    string nickname,
    string description,
    string id,
    GH_Exposure exposure = GH_Exposure.primary)
    : GH_Param<TGoo>($"{nickname} Parameter", nickname, description, "Robots", "Parameters", GH_ParamAccess.item)
    where TGoo : Goo<T, TGoo>, new()
{
    public override GH_Exposure Exposure => exposure;
    public override Guid ComponentGuid => new(id);
    protected override Bitmap Icon => Util.GetIcon(GetType());

    protected override TGoo PreferredCast(object data) =>
        data is T value ? New(value) : CastFailed();

    internal static TGoo New(T value)
    {
        var goo = new TGoo();
        goo.SetValue(value);
        return goo;
    }

    static TGoo CastFailed() => null!;
}

public abstract class PersistentParam<T, TGoo>(
    string nickname,
    string description,
    string id,
    GH_Exposure exposure = GH_Exposure.primary)
    : GH_PersistentParam<TGoo>($"{nickname} Parameter", nickname, description, "Robots", "Parameters")
    where TGoo : Goo<T, TGoo>, new()
{
    public override GH_Exposure Exposure => exposure;
    public override Guid ComponentGuid => new(id);
    protected override Bitmap Icon => Util.GetIcon(GetType());

    protected override TGoo PreferredCast(object data) =>
        data is T value ? Param<T, TGoo>.New(value) : CastFailed();

    protected override GH_GetterResult Prompt_Singular(ref TGoo value)
    {
        value = new TGoo();
        return GH_GetterResult.success;
    }

    protected override GH_GetterResult Prompt_Plural(ref List<TGoo> values)
    {
        values = [];
        return GH_GetterResult.success;
    }

    static TGoo CastFailed() => null!;
}
