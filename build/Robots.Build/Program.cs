using Robots.Build;

var commandList = new Dictionary<string, Action>
{
    { "build", Commands.Build },
    { "package", Commands.Package },
    { "publish", Commands.Publish },
};

if (args.Length == 0)
{
    Console.WriteLine($"Specify a command: {string.Join(", ", commandList.Keys)}");
    return 1;
}

foreach (var arg in args)
{
    if (!commandList.TryGetValue(arg, out var action))
    {
        Console.WriteLine($"Command '{arg}' doesn't exist.");
        return 1;
    }

    Console.WriteLine($"Starting {arg}...");
    action();
}

Console.WriteLine("Finished with no errors.");
return 0;