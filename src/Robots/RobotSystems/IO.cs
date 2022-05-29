
namespace Robots;

public class IO
{
    private readonly Manufacturers _manufacturer;
    public string[] DO { get; }
    public string[] DI { get; }
    public string[] AO { get; }
    public string[] AI { get; }
    public bool UseControllerNumbering { get; }

    public IO(Manufacturers manufacturer, bool useControllerNumbering, string[] @do, string[] di, string[] ao, string[] ai)
    {
        _manufacturer = manufacturer;
        UseControllerNumbering = useControllerNumbering;
        DO = @do;
        DI = di;
        AO = ao;
        AI = ai;
    }

    internal void CheckBounds(int index, string[] array)
    {
        if (UseControllerNumbering)
        {
            if (index < GetStartIndex())
                throw new ArgumentOutOfRangeException(nameof(index), " Index of IO is out of range.");

            return;
        }

        if (index < 0 || index >= array.Length)
            throw new ArgumentOutOfRangeException(nameof(index), " Index of IO is out of range.");
    }

    int GetStartIndex() => _manufacturer switch
    {
        Manufacturers.ABB => 1,
        Manufacturers.KUKA => 1,
        _ => 0
    };
}