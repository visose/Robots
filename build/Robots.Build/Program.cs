using Robots.Build;
using static Robots.Build.Util;

var commandList = new Dictionary<string, Delegate>
{
    { "test", Commands.Test },
    { "check", Commands.CheckVersionAsync },
    { "build", Commands.Build },
    { "package", Commands.PackageAsync },
    { "publish", Commands.PublishAsync },
    { "release", Commands.ReleaseAsync }
};

if (args.Length == 0)
{
    Log($"Specify a command: {string.Join(", ", commandList.Keys)}");
    return 1;
}

foreach (var arg in args)
{
    if (!commandList.TryGetValue(arg, out var action))
    {
        Log($"Command '{arg}' doesn't exist.");
        return 1;
    }

    Log($"Starting {arg}...");

    var result = action.DynamicInvoke().NotNull();

    int code = action.Method.ReturnType == typeof(int)
        ? (int)result
        : await (Task<int>)result;

    if (code != 0)
    {
        Log($"Stopped at step: {arg}");
        return Math.Max(code, 0);
    }
}

Console.WriteLine("Finished with no errors.");
return 0;