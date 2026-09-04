using System.Globalization;
using static System.Math;

namespace Robots;

static class Util
{
    // Constants

    public const double DistanceTol = 0.001;
    public const double AngleTol = 0.001;
    public const double TimeTol = 0.00001;
    public const double UnitTol = 0.000001;
    public const double SingularityTol = 0.0001;
    public const double HalfPI = PI * 0.5;
    public const double PI2 = PI * 2.0;

    // Exceptions

    extension<T>(T? source)
    {
        public T NotNull(string? text = null)
        {
            return source ?? throw new ArgumentNullException(null, text);
        }
    }

    public static double CheckFinite(double value, string name, string message = "Value must be finite.")
    {
        return double.IsFinite(value) ? value : throw new ArgumentException(message, name);
    }

    public static T CheckFinite<T>(T values, string name, string message) where T : IReadOnlyList<double>
    {
        for (int i = 0; i < values.Count; i++)
        {
            if (!double.IsFinite(values[i]))
                throw new ArgumentException(message, name);
        }

        return values;
    }

    public static NotSupportedException Unsupported<T>(T value) => new($"{typeof(T).Name} '{value}' is not supported.");

    // String

    extension(string? a)
    {
        public bool EqualsIgnoreCase(string? b)
        {
            return string.Equals(a, b, StringComparison.OrdinalIgnoreCase);
        }

        public string UseCRLF()
        {
            ArgumentNullException.ThrowIfNull(a);
            return a.ReplaceLineEndings("\r\n");
        }

        public string UseLF()
        {
            ArgumentNullException.ThrowIfNull(a);
            return a.ReplaceLineEndings("\n");
        }
    }

    extension(int value)
    {
        public string Text() => value.ToString(CultureInfo.InvariantCulture);
    }

    extension(TimeSpan value)
    {
        public string Text(string format) => value.ToString(format, CultureInfo.InvariantCulture);
    }

    // Collection

    extension<T>(IReadOnlyList<T> array)
    {
        public List<K> MapToList<K>(Func<T, K> projection)
        {
            var result = new List<K>(array.Count);

            for (int i = 0; i < array.Count; i++)
                result.Add(projection(array[i]));

            return result;
        }

        public K[] Map<K>(Func<T, K> projection)
        {
            var result = new K[array.Count];

            for (int i = 0; i < array.Count; i++)
                result[i] = projection(array[i]);

            return result;
        }

        public K[] Map<K>(Func<T, int, K> projection)
        {
            var result = new K[array.Count];

            for (int i = 0; i < array.Count; i++)
                result[i] = projection(array[i], i);

            return result;
        }

        public K[] FlattenToArray<K>(Func<T, IReadOnlyList<K>> selector)
        {
            int count = 0;

            for (int i = 0; i < array.Count; i++)
                count += selector(array[i]).Count;

            var result = new K[count];
            int index = 0;

            for (int i = 0; i < array.Count; i++)
            {
                var values = selector(array[i]);

                for (int j = 0; j < values.Count; j++)
                    result[index++] = values[j];
            }

            return result;
        }

        public T MaxBy<K>(Func<T, K> comparable) where K : IComparable<K>
        {
            ArgumentOutOfRangeException.ThrowIfZero(array.Count, nameof(array));

            T maxItem = array[0];
            K maxValue = comparable(maxItem);

            for (int i = 1; i < array.Count; i++)
            {
                var item = array[i];
                var val = comparable(item);

                if (val.CompareTo(maxValue) > 0)
                {
                    maxItem = item;
                    maxValue = val;
                }
            }

            return maxItem;
        }
    }

    extension<T>(T[] array)
    {
        public T[] Subset(int[] indices)
        {
            T[] subset = new T[indices.Length];

            for (int i = 0; i < indices.Length; i++)
                subset[i] = array[indices[i]];

            return subset;
        }

    }

    extension<T>(IReadOnlyList<IReadOnlyList<T>> source)
    {
        public List<T>[] Transpose()
        {
            if (source.Count == 0)
                return [];

            int count = source[0].Count;

            for (int i = 0; i < source.Count; i++)
                ArgumentOutOfRangeException.ThrowIfNotEqual(source[i].Count, count, nameof(source));

            var result = new List<T>[count];

            for (int i = 0; i < count; i++)
            {
                var row = new List<T>(source.Count);

                for (int j = 0; j < source.Count; j++)
                    row.Add(source[j][i]);

                result[i] = row;
            }

            return result;
        }
    }

}
