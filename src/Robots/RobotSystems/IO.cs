
namespace Robots;

public class IO(Manufacturers manufacturer, bool useControllerNumbering, string[] @do, string[] di, string[] ao, string[] ai)
{
    private readonly Manufacturers _manufacturer = manufacturer;
    public string[] DO { get; } = @do;
    public string[] DI { get; } = di;
    public string[] AO { get; } = ao;
    public string[] AI { get; } = ai;
    public bool UseControllerNumbering { get; } = useControllerNumbering;

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