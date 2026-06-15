using System.Globalization;
using System.Reflection;
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

    extension(Exception)
    {
        public static void ThrowIfNotEqual(int actual, int expected, string message)
        {
            if (actual != expected)
                throw new ArgumentException(message);
        }
    }

    public static NotSupportedException Unsupported<T>(T value) => new($"{typeof(T).Name} '{value}' is not supported.");

    // Resources

    public static StreamReader GetResource(string name)
    {
        var assembly = Assembly.GetExecutingAssembly();
        var resourceName = $"Robots.Resources.Embedded.{name}";
        var stream = assembly.GetManifestResourceStream(resourceName)
            ?? throw new FileNotFoundException($"Embedded resource '{resourceName}' was not found.");

        return new(stream);
    }

    public static string GetStringResource(string name)
    {
        using var reader = GetResource(name);
        return reader.ReadToEnd();
    }

    // String

    extension(string? a)
    {
        public bool EqualsIgnoreCase(string? b)
        {
            return string.Equals(a, b, StringComparison.OrdinalIgnoreCase);
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

        internal K[] FlattenToArray<K>(Func<T, IReadOnlyList<K>> selector)
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

        public T[] RangeSubset(int startIndex, int length)
        {
            T[] subset = new T[length];
            Array.Copy(array, startIndex, subset, 0, length);
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

    public static byte[] Combine(params byte[][] arrays)
    {
        var result = new byte[arrays.Sum(x => x.Length)];
        int offset = 0;

        foreach (byte[] data in arrays)
        {
            Buffer.BlockCopy(data, 0, result, offset, data.Length);
            offset += data.Length;
        }

        return result;
    }

    // Matrix

    public static double[,] Mult(this double[,] a, double[,] b)
    {
        int rA = a.GetLength(0);
        int cA = a.GetLength(1);
        int rB = b.GetLength(0);
        int cB = b.GetLength(1);

        if (cA != rB)
            throw new("Matrices have incompatible dimensions.");

        var result = new double[rA, cB];

        for (int i = 0; i < rA; i++)
        {
            for (int j = 0; j < cB; j++)
            {
                double n = 0;

                for (int k = 0; k < cA; k++)
                    n += a[i, k] * b[k, j];

                result[i, j] = n;
            }
        }

        return result;
    }

    extension(double[] a)
    {
        public double[] Mult(double[,] b)
        {
            int cA = a.Length;
            int rB = b.GetLength(0);
            int cB = b.GetLength(1);

            if (cA != rB)
                throw new("Matrices have incompatible dimensions.");

            var result = new double[cB];

            for (int j = 0; j < cB; j++)
            {
                double n = 0;

                for (int k = 0; k < cA; k++)
                    n += a[k] * b[k, j];

                result[j] = n;
            }

            return result;
        }
    }

    public static double[,] Transpose(this double[,] matrix)
    {
        int w = matrix.GetLength(0);
        int h = matrix.GetLength(1);

        var result = new double[h, w];

        for (int i = 0; i < w; i++)
        {
            for (int j = 0; j < h; j++)
                result[j, i] = matrix[i, j];
        }

        return result;
    }
}
