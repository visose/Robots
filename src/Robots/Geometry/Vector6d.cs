
namespace Robots;

public struct Vector6d
{
    // static

    public static Vector6d Map<T>(T[] array, Func<T, double> projection)
    {
        if (array.Length != 6)
            throw new ArgumentOutOfRangeException(nameof(array), " Array length should be 6");

        Vector6d result = default;

        for (int i = 0; i < 6; i++)
            result[i] = projection(array[i]);

        return result;
    }

    // instance

    public double A1;
    public double A2;
    public double A3;
    public double A4;
    public double A5;
    public double A6;

    public Vector6d(double a1, double a2, double a3, double a4, double a5, double a6)
    {
        A1 = a1;
        A2 = a2;
        A3 = a3;
        A4 = a4;
        A5 = a5;
        A6 = a6;
    }

    public Vector6d(double[] joints)
    {
        if (joints.Length != 6)
            throw new ArgumentOutOfRangeException(nameof(joints), " Array length should be 6");

        A1 = joints[0];
        A2 = joints[1];
        A3 = joints[2];
        A4 = joints[3];
        A5 = joints[4];
        A6 = joints[5];
    }

    public double this[int i]
    {
        readonly get => i switch
        {
            0 => A1,
            1 => A2,
            2 => A3,
            3 => A4,
            4 => A5,
            5 => A6,
            _ => throw new ArgumentOutOfRangeException(nameof(i), " Invalid Vector6 index"),
        };

        set
        {
            switch (i)
            {
                case 0: A1 = value; break;
                case 1: A2 = value; break;
                case 2: A3 = value; break;
                case 3: A4 = value; break;
                case 4: A5 = value; break;
                case 5: A6 = value; break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(i), " Invalid Vector6 index");
            }
        }
    }

    public readonly double[] ToArray()
    {
        return [A1, A2, A3, A4, A5, A6];
    }
}
