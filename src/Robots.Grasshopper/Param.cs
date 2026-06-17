using System.Drawing;

using Rhino.Geometry;

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

    public override void CreateAttributes()
    {
        m_attributes = new ParameterAttributes(this);
    }

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

public abstract class PreviewParam<T, TGoo>(
    string nickname,
    string description,
    string id,
    GH_Exposure exposure = GH_Exposure.primary)
    : Param<T, TGoo>(nickname, description, id, exposure),
    IGH_PreviewObject
    where TGoo : Goo<T, TGoo>, IGH_PreviewData, new()
{
    public bool Hidden { get; set; }
    public bool IsPreviewCapable => true;
    public BoundingBox ClippingBox => Preview_ComputeClippingBox();
    public void DrawViewportWires(IGH_PreviewArgs args) => DrawViewportMeshes(args);
    public void DrawViewportMeshes(IGH_PreviewArgs args) => Preview_DrawMeshes(args);
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

    public override void CreateAttributes()
    {
        m_attributes = new ParameterAttributes(this);
    }

    protected override TGoo PreferredCast(object data) =>
        data is T value ? Param<T, TGoo>.New(value) : CastFailed();

    protected override GH_GetterResult Prompt_Singular(ref TGoo value)
    {
        return GH_GetterResult.cancel;
    }

    protected override GH_GetterResult Prompt_Plural(ref List<TGoo> values)
    {
        return GH_GetterResult.cancel;
    }

    static TGoo CastFailed() => null!;
}
