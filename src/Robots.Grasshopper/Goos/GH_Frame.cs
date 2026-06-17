using GH_IO.Serialization;
using Rhino.Geometry;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Frame() : Goo<Frame, GH_Frame>("Frame", Frame.Default)
{
    const string DataKey = "Data";
    const string PlaneKey = "Plane";

    readonly record struct FrameData(int CoupledMechanism, int CoupledMechanicalGroup, bool UseController, string? Name, int? Number)
    {
        internal static FrameData From(Frame frame) =>
            new(frame.CoupledMechanism, frame.CoupledMechanicalGroup, frame.UseController, frame.HasName ? frame.Name : null, frame.Number);

        internal Frame ToFrame(Plane plane) =>
            new(plane, CoupledMechanism, CoupledMechanicalGroup, Name, UseController, Number);
    }

    public override bool CastFrom(object source)
    {
        if (base.CastFrom(source))
            return true;

        switch (source)
        {
            case GH_Plane plane:
                Value = new(plane.Value);
                return true;
            case GH_Point point:
                Value = new(new(point.Value, Vector3d.XAxis, Vector3d.YAxis));
                return true;
            default:
                return false;
        }
    }

    public override bool CastTo<TGoo>(ref TGoo target)
    {
        if (base.CastTo(ref target))
            return true;

        if (Value is not null && typeof(TGoo) == typeof(GH_Plane))
        {
            target = (TGoo)(object)new GH_Plane(Value.Plane);
            return true;
        }

        return false;
    }

    public override bool Write(GH_IWriter writer)
    {
        if (Value is null)
            return false;

        writer.SetPlane(PlaneKey, GooSerialization.ToGhPlane(Value.Plane));
        writer.SetJson(DataKey, FrameData.From(Value));

        return true;
    }

    public override bool Read(GH_IReader reader)
    {
        if (!reader.ItemExists(PlaneKey) || !reader.TryGetJson<FrameData>(DataKey, out var data))
            return false;

        Value = data.ToFrame(GooSerialization.ToPlane(reader.GetPlane(PlaneKey)));
        return true;
    }

    protected override Frame DuplicateValue(Frame value) =>
        FrameData.From(value).ToFrame(value.Plane);
}
