
namespace Robots;

public class IO
{
    public string[] DO { get; }
    public string[] DI { get; }
    public string[] AO { get; }
    public string[] AI { get; }

    public IO(string[] @do, string[] di, string[] ao, string[] ai)
    {
        DO = @do;
        DI = di;
        AO = ao;
        AI = ai;
    }
}