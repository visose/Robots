using Robots.Icons;

try
{
    var options = Options.Parse(args);
    var root = FindRepositoryRoot();
    var outputRoot = options.OutputRoot ?? Path.Combine(root, "artifacts", "robots-icons-skia");
    var iconsDirectory = Path.Combine(outputRoot, "icons");
    var contactSheet = Path.Combine(outputRoot, "contact-sheet.png");

    _ = Directory.CreateDirectory(iconsDirectory);

    var outputs = IconSet.WriteIcons(iconsDirectory);
    IconQa.ThrowIfInvalid(outputs);
    ContactSheet.Write(outputs, contactSheet);

    Console.WriteLine($"Wrote {outputs.Count} icons to {iconsDirectory}");
    Console.WriteLine($"Wrote contact sheet to {contactSheet}");
    return 0;
}
catch (Exception exception)
{
    Console.Error.WriteLine(exception.Message);
    return 1;
}

static string FindRepositoryRoot()
{
    var current = new DirectoryInfo(Directory.GetCurrentDirectory());

    while (current is not null)
    {
        var globalJson = Path.Combine(current.FullName, "global.json");
        var grasshopperProject = Path.Combine(current.FullName, "src", "Robots.Grasshopper", "Robots.Grasshopper.csproj");

        if (File.Exists(globalJson) && File.Exists(grasshopperProject))
        {
            return current.FullName;
        }

        current = current.Parent;
    }

    throw new InvalidOperationException("Could not find the Robots repository root. Run this tool from inside the repository.");
}

file sealed record Options(string? OutputRoot)
{
    public static Options Parse(string[] args)
    {
        string? outputRoot = null;

        for (var i = 0; i < args.Length; i++)
        {
            if (args[i] == "--output")
            {
                if (outputRoot is not null)
                {
                    throw new ArgumentException("The --output option can only be supplied once.");
                }

                if (i + 1 >= args.Length)
                {
                    throw new ArgumentException("The --output option requires a path.");
                }

                outputRoot = Path.GetFullPath(args[++i]);
                continue;
            }

            throw new ArgumentException($"Unknown argument: {args[i]}");
        }

        return new Options(outputRoot);
    }
}
