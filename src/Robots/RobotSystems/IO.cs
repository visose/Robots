namespace Robots;

public class IO
{
    public string[] DO { get; }
    public string[] DI { get; }
    public string[] AO { get; }
    public string[] AI { get; }

    public IO(string[]? @do = null, string[]? di = null, string[]? ao = null, string[]? ai = null)
    {
        DO = @do ?? new string[0];
        DI = di ?? new string[0];
        AO = ao ?? new string[0];
        AI = ai ?? new string[0];
    }
}
