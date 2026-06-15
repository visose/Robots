using System.Globalization;

using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

static class DataAccess
{
    extension(IGH_DataAccess DA)
    {
        internal T Get<T>(int index) { T value = default!; return Required(DA.GetData(index, ref value), value); }

        internal T Get<T>(string name) { T value = default!; return Required(DA.GetData(name, ref value), value); }

        internal T Get<T>(int index, T fallback) { T value = fallback; return DA.GetData(index, ref value) && value is not null ? value : fallback; }

        internal T Get<T>(string name, T fallback) { T value = fallback; return DA.GetData(name, ref value) && value is not null ? value : fallback; }

        internal T? Maybe<T>(int index) where T : class { T? value = null; return DA.GetData(index, ref value) ? value : null; }

        internal T? Maybe<T>(string name) where T : class { T? value = null; return DA.GetData(name, ref value) ? value : null; }

        internal T? MaybeValue<T>(int index) where T : struct { T value = default; return DA.GetData(index, ref value) ? value : null; }

        internal T? MaybeValue<T>(string name) where T : struct { T value = default; return DA.GetData(name, ref value) ? value : null; }

        internal T[] List<T>(int index) { List<T> values = []; return DA.GetDataList(index, values) ? values.CheckedArray(index.ToString(CultureInfo.InvariantCulture)) : throw new MissingInputException(); }

        internal T[] List<T>(string name) { List<T> values = []; return DA.GetDataList(name, values) ? values.CheckedArray(name) : throw new MissingInputException(); }

        internal T[] MaybeList<T>(int index) { List<T> values = []; return DA.GetDataList(index, values) ? values.CheckedArray(index.ToString(CultureInfo.InvariantCulture)) : []; }

        internal T[] MaybeList<T>(string name) { List<T> values = []; return DA.GetDataList(name, values) ? values.CheckedArray(name) : []; }

        internal GH_Structure<T> Tree<T>(int index)
            where T : IGH_Goo
        {
            return DA.GetDataTree(index, out GH_Structure<T> tree) ? tree : throw new MissingInputException();
        }

        internal List<List<string>> TextTree(int index)
        {
            return [.. DA.Tree<GH_String>(index).Branches.Select(branch => branch.Select(goo => goo.Value).ToList())];
        }

        internal void SetTextTree(int index, IReadOnlyList<IReadOnlyList<IReadOnlyList<string>>> values, GH_Path path)
        {
            var tree = TextTree();

            for (int i = 0; i < values.Count; i++)
            {
                var tempPath = path.AppendElement(i);
                for (int j = 0; j < values[i].Count; j++)
                    tree.AppendText(values[i][j], tempPath.AppendElement(j));
            }

            _ = DA.SetDataTree(index, tree);
        }
    }

    internal static GH_Structure<GH_String> TextTree() => [];

    internal static GH_Structure<GH_Number> NumberTree() => [];

    internal static GH_Structure<GH_Plane> PlaneTree() => [];

    extension(GH_Structure<GH_String> tree)
    {
        internal void AppendText(IReadOnlyList<string> values, GH_Path path)
        {
            for (int i = 0; i < values.Count; i++)
                tree.Append(new(values[i]), path);
        }

        internal void AppendText(string value, GH_Path path) => tree.Append(new(value), path);
    }

    extension(GH_Structure<GH_Number> tree)
    {
        internal void AppendNumbers(IReadOnlyList<double> values, GH_Path path)
        {
            for (int i = 0; i < values.Count; i++)
                tree.Append(new(values[i]), path);
        }

        internal void AppendNumber(double value, GH_Path path) => tree.Append(new(value), path);
    }

    extension(GH_Structure<GH_Plane> tree)
    {
        internal void AppendPlanes(IReadOnlyList<Rhino.Geometry.Plane> values, GH_Path path)
        {
            for (int i = 0; i < values.Count; i++)
                tree.Append(new(values[i]), path);
        }
    }

    static T Required<T>(bool success, T? value) => success && value is not null ? value : throw new MissingInputException();

    extension<T>(List<T> values)
    {
        T[] CheckedArray(string input)
        {
            if (values.Any(value => value is null))
                throw new InvalidOperationException($"Input '{input}' contains null values.");

            return [.. values];
        }
    }
}
