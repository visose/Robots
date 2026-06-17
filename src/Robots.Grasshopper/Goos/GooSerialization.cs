using System.Text.Json;
using GH_IO.Serialization;
using GhPlane = GH_IO.Types.GH_Plane;
using Rhino.Geometry;

namespace Robots.Grasshopper;

static class GooSerialization
{
    static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true
    };

    internal static void SetJson<T>(this GH_IWriter writer, string key, T value) =>
        writer.SetString(key, JsonSerializer.Serialize(value, JsonOptions));

    internal static bool TryGetJson<T>(this GH_IReader reader, string key, out T value)
    {
        value = default!;

        if (!reader.ItemExists(key))
            return false;

        try
        {
            var result = JsonSerializer.Deserialize<T>(reader.GetString(key), JsonOptions);

            if (result is null)
                return false;

            value = result;
            return true;
        }
        catch (JsonException)
        {
            return false;
        }
    }

    internal static GhPlane ToGhPlane(Plane plane) => new(
        plane.OriginX,
        plane.OriginY,
        plane.OriginZ,
        plane.XAxis.X,
        plane.XAxis.Y,
        plane.XAxis.Z,
        plane.YAxis.X,
        plane.YAxis.Y,
        plane.YAxis.Z);

    internal static Plane ToPlane(GhPlane plane) => new(
        new Point3d(plane.Origin.x, plane.Origin.y, plane.Origin.z),
        new Vector3d(plane.XAxis.x, plane.XAxis.y, plane.XAxis.z),
        new Vector3d(plane.YAxis.x, plane.YAxis.y, plane.YAxis.z));
}
