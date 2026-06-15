
namespace Robots;

public class IO(Manufacturers manufacturer, bool useControllerNumbering, string[] @do, string[] di, string[] ao, string[] ai)
{
    readonly int _controllerStartIndex = useControllerNumbering ? GetStartIndex(manufacturer) : 0;

    public string[] DO { get; } = @do;
    public string[] DI { get; } = di;
    public string[] AO { get; } = ao;
    public string[] AI { get; } = ai;
    public bool UseControllerNumbering { get; } = useControllerNumbering;

    internal string? ValidateBounds(int index, string[] array)
    {
        if (UseControllerNumbering)
            return index < _controllerStartIndex ? "IO index is out of range." : null;

        return index < 0 || index >= array.Length ? "IO index is out of range." : null;
    }

    static int GetStartIndex(Manufacturers manufacturer)
    {
        if (manufacturer is Manufacturers.ABB or Manufacturers.KUKA)
            return 1;

        throw new NotSupportedException($"Controller IO numbering is not supported for {manufacturer} robots.");
    }
}
