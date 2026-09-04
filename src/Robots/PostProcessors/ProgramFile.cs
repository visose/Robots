using System.Text;

namespace Robots;

static class ProgramFile
{
    public static Encoding Utf8WithBom { get; } = new UTF8Encoding(true);

    public static List<List<List<string>>> RequireCode(IProgram program) =>
        program.Code ?? throw new InvalidOperationException("Program code was not generated.");

    public static void ValidateFolder(string folder) => ArgumentException.ThrowIfNullOrWhiteSpace(folder);

    public static string CreateDirectory(string folder, string programName)
    {
        string path = Path.Combine(folder, programName);
        _ = Directory.CreateDirectory(path);
        return path;
    }

    public static void WriteText(string file, string text, Encoding? encoding = null)
    {
        if (encoding is null)
            File.WriteAllText(file, text);
        else
            File.WriteAllText(file, text, encoding);
    }

    public static void WriteCode(
        string file,
        IEnumerable<string> code,
        string lineEnding = "\r\n",
        Encoding? encoding = null,
        bool trailingNewline = false)
    {
        string text = string.Join(lineEnding, code);

        if (trailingNewline)
            text += lineEnding;

        WriteText(file, text, encoding);
    }
}

public static class PythonProgramFile
{
    public static void Save(IProgram program, string folder)
    {
        ProgramFile.ValidateFolder(folder);
        var code = ProgramFile.RequireCode(program);
        string file = Path.Combine(folder, $"{program.Name}.py");
        ProgramFile.WriteCode(file, code[0][0], "\n");
    }
}
