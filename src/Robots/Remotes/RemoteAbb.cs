using System.Diagnostics;

namespace Robots;

public class RemoteAbb : IRemote
{
    static readonly TimeSpan DefaultTimeout = TimeSpan.FromSeconds(30);

    readonly IAbbRemoteProcessRunner _runner;
    readonly string _helperPath;
    readonly TimeSpan _timeout;

    public string? IP { get; set; }
    public List<string> Log { get; } = [];

    internal RemoteAbb()
        : this(GetDefaultHelperPath(), new AbbRemoteProcessRunner(), DefaultTimeout)
    { }

    internal RemoteAbb(string helperPath, IAbbRemoteProcessRunner runner, TimeSpan timeout)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(helperPath);

        if (timeout <= TimeSpan.Zero)
            throw new ArgumentOutOfRangeException(nameof(timeout), "Timeout must be greater than zero.");

        _helperPath = helperPath;
        _runner = runner;
        _timeout = timeout;
    }

    public void Play() => Send(AbbRemoteActions.Play, null);

    public void Pause() => Send(AbbRemoteActions.Pause, null);

    public void Upload(IProgram program)
    {
        if (program.Code is null)
            throw new InvalidOperationException("ABB program code was not generated.");

        if (program.Code.Count != 1)
            throw new NotSupportedException("ABB remote upload currently supports one RAPID task, T_ROB1.");

        if (!Program.IsValidIdentifier(program.Name, out var error))
            throw new InvalidOperationException($"ABB program {error}");

        string tempFolder = CreateTempFolder();

        try
        {
            program.Save(tempFolder);

            string localFolder = Path.Combine(tempFolder, program.Name);

            if (!Directory.Exists(localFolder))
                throw new InvalidOperationException($"ABB program folder was not generated: {localFolder}");

            Send(AbbRemoteActions.Upload, new(program.Name, localFolder));
        }
        finally
        {
            DeleteTempFolder(tempFolder);
        }
    }

    void Send(string action, AbbRemoteUpload? upload)
    {
        if (!OperatingSystem.IsWindows())
            throw new PlatformNotSupportedException("ABB remote control is only available on Windows.");

        string? ip = NormalizeIp(IP);
        string id = Guid.NewGuid().ToString("N");
        AbbRemoteRequest request = new(id, action, ip, upload?.ProgramName, upload?.LocalFolder);

        if (ip is null)
            AddLog("No ABB controller IP was set; the helper will use the first available controller.");

        AddLog($"Starting ABB remote {action} helper.");

        try
        {
            var response = _runner.Run(_helperPath, request, _timeout);
            AddResponseLog(response);

            if (!response.Ok)
            {
                string code = string.IsNullOrWhiteSpace(response.ErrorCode) ? AbbRemoteErrorCodes.Unsupported : response.ErrorCode;
                throw new AbbRemoteException(response.Message, code);
            }
        }
        catch (Exception exception) when (exception is not AbbRemoteException)
        {
            AddLog($"Error: {exception.Message}");
            throw;
        }
    }

    void AddResponseLog(AbbRemoteResponse response)
    {
        foreach (string entry in response.Log)
            AddLog(entry);

        if (!string.IsNullOrWhiteSpace(response.Message))
            AddLog(response.Message);
    }

    void AddLog(string text)
    {
        Log.Insert(0, $"{DateTime.Now:T} - {text}");
    }

    static string? NormalizeIp(string? ip)
    {
        if (string.IsNullOrWhiteSpace(ip))
            return null;

        string trimmed = ip.Trim();

        if (!System.Net.IPAddress.TryParse(trimmed, out _))
            throw new InvalidOperationException($"ABB controller IP address is invalid: {trimmed}");

        return trimmed;
    }

    static string CreateTempFolder()
    {
        string folder = Path.Combine(Path.GetTempPath(), "Robots", "abb-remote", Guid.NewGuid().ToString("N"));
        _ = Directory.CreateDirectory(folder);
        return folder;
    }

    static void DeleteTempFolder(string folder)
    {
        string fullPath = Path.GetFullPath(folder);
        string root = Path.GetFullPath(Path.Combine(Path.GetTempPath(), "Robots", "abb-remote"));

        string rootPrefix = Path.EndsInDirectorySeparator(root)
            ? root
            : root + Path.DirectorySeparatorChar;

        if (!fullPath.StartsWith(rootPrefix, StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException($"Refusing to delete ABB remote temp folder outside {root}.");

        if (Directory.Exists(fullPath))
            Directory.Delete(fullPath, true);
    }

    static string GetDefaultHelperPath()
    {
        string? configured = Environment.GetEnvironmentVariable("ROBOTS_ABB_REMOTE_HELPER");

        if (!string.IsNullOrWhiteSpace(configured))
            return configured;

        return Path.Combine(AppContext.BaseDirectory, "abb-remote", "Robots.AbbRemote.exe");
    }
}

internal readonly record struct AbbRemoteUpload(string ProgramName, string LocalFolder);

internal interface IAbbRemoteProcessRunner
{
    AbbRemoteResponse Run(string helperPath, AbbRemoteRequest request, TimeSpan timeout);
}

internal sealed class AbbRemoteProcessRunner : IAbbRemoteProcessRunner
{
    readonly IAbbRemoteProcessFactory _factory;

    public AbbRemoteProcessRunner()
        : this(new AbbRemoteProcessFactory())
    { }

    internal AbbRemoteProcessRunner(IAbbRemoteProcessFactory factory)
    {
        _factory = factory;
    }

    public AbbRemoteResponse Run(string helperPath, AbbRemoteRequest request, TimeSpan timeout)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(helperPath);

        if (!File.Exists(helperPath))
            throw new FileNotFoundException($"ABB remote helper was not found: {helperPath}", helperPath);

        string requestJson = AbbRemoteProtocol.SerializeRequest(request);

        using IAbbRemoteProcess process = _factory.Start(helperPath);
        Task<string?> stdoutTask = process.StandardOutput.ReadLineAsync();
        Task<string> stderrTask = process.StandardError.ReadToEndAsync();

        process.StandardInput.WriteLine(requestJson);
        process.StandardInput.Flush();
        process.StandardInput.Dispose();

        int timeoutMilliseconds = timeout.TotalMilliseconds >= int.MaxValue
            ? int.MaxValue
            : (int)timeout.TotalMilliseconds;

        if (!process.WaitForExit(timeoutMilliseconds))
        {
            process.Kill();
            _ = process.WaitForExit(1000);
            throw new TimeoutException($"ABB remote helper timed out after {timeout.TotalSeconds:N0} seconds.");
        }

        string stderr = stderrTask.GetAwaiter().GetResult();
        string? stdout = stdoutTask.GetAwaiter().GetResult();

        if (process.ExitCode != 0)
        {
            string detail = string.IsNullOrWhiteSpace(stderr) ? "No stderr output." : stderr.Trim();
            throw new InvalidOperationException($"ABB remote helper exited with code {process.ExitCode}: {detail}");
        }

        if (!string.IsNullOrWhiteSpace(stderr))
            throw new InvalidOperationException($"ABB remote helper wrote to stderr: {stderr.Trim()}");

        if (string.IsNullOrWhiteSpace(stdout))
            throw new InvalidOperationException("ABB remote helper returned an empty response.");

        AbbRemoteResponse response = AbbRemoteProtocol.DeserializeResponse(stdout);

        if (!string.Equals(response.Id, request.Id, StringComparison.Ordinal))
            throw new InvalidOperationException("ABB remote helper response id did not match the request id.");

        return response;
    }
}

internal interface IAbbRemoteProcessFactory
{
    IAbbRemoteProcess Start(string helperPath);
}

internal interface IAbbRemoteProcess : IDisposable
{
    TextWriter StandardInput { get; }
    TextReader StandardOutput { get; }
    TextReader StandardError { get; }
    int ExitCode { get; }
    bool WaitForExit(int milliseconds);
    void Kill();
}

internal sealed class AbbRemoteProcessFactory : IAbbRemoteProcessFactory
{
    public IAbbRemoteProcess Start(string helperPath)
    {
        Process process = new()
        {
            StartInfo = new()
            {
                FileName = helperPath,
                UseShellExecute = false,
                RedirectStandardInput = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            }
        };

        if (!process.Start())
            throw new InvalidOperationException($"ABB remote helper could not be started: {helperPath}");

        return new AbbRemoteProcess(process);
    }
}

internal sealed class AbbRemoteProcess(Process process) : IAbbRemoteProcess
{
    public TextWriter StandardInput => process.StandardInput;
    public TextReader StandardOutput => process.StandardOutput;
    public TextReader StandardError => process.StandardError;
    public int ExitCode => process.ExitCode;

    public bool WaitForExit(int milliseconds) => process.WaitForExit(milliseconds);

    public void Kill()
    {
        try
        {
            process.Kill(entireProcessTree: true);
        }
        catch (InvalidOperationException)
        { }
    }

    public void Dispose() => process.Dispose();
}

internal sealed class AbbRemoteException(string message, string errorCode, Exception? innerException = null)
    : InvalidOperationException(message, innerException)
{
    public string ErrorCode { get; } = errorCode;
}
