using System.Text;
using Renci.SshNet;

namespace Robots;

public class RemoteFranka : IRemote, IRemoteNotifier, IDisposable
{
    readonly object _sync = new();
    User? _user;
    string? _uploadedFile;
    bool _disposed;

    public Action? Update { get; set; }
    public List<string> Log { get; } = [];

    CancellationTokenSource? _cancelToken;
    Task? _task;

    public string? IP
    {
        get => _user?.IP;
        set
        {
            if (string.IsNullOrWhiteSpace(value))
            {
                _user = null;
                LogAdd("Invalid address.");
                return;
            }

            try
            {
                _user = new(value);
            }
            catch
            {
                _user = null;
                LogAdd($"Invalid address: {value}.");
            }
        }
    }

    public void Upload(IProgram program)
    {
        if (_user is null)
        {
            LogAdd("Error: IP is not set.");
            return;
        }

        if (program.Code is null)
        {
            LogAdd("Error: Program code was not generated.");
            return;
        }

        try
        {
            var code = string.Join("\n", program.Code[0].SelectMany(c => c));
            var bytes = Encoding.UTF8.GetBytes(code);
            string fileName = $"{program.Name}.py";
            Ftp.Upload(bytes, fileName, _user);
            _uploadedFile = fileName;
            LogAdd("Program uploaded.");

        }
        catch (Exception e)
        {
            LogAdd($"Error: FTP - {e.Message}");
        }
    }

    public void Pause()
    {
        lock (_sync)
        {
            if (_cancelToken is null)
            {
                LogAdd("Program not running.");
                return;
            }

            _cancelToken.Cancel();
        }
    }

    public void Play()
    {
        if (_user is null)
        {
            LogAdd("Error: IP is not set.");
            return;
        }

        if (_uploadedFile is null)
        {
            LogAdd("Error: File was not uploaded.");
            return;
        }

        Send($"echo -e '{_user.Password}\n' | sudo -S python3 -u {_user.ProgramsDir}/{_uploadedFile}");
    }

    public void Send(string message)
    {
        lock (_sync)
        {
            ObjectDisposedException.ThrowIf(_disposed, this);

            if (_task?.IsCompleted == false)
                return;

            if (_user is not { } user)
            {
                LogAdd("Error: IP is not set.");
                return;
            }

            CancellationTokenSource cancellation = new();
            _cancelToken = cancellation;
            _task = Task.Run(() => Run(message, user, cancellation));
        }
    }

    async Task Run(string message, User user, CancellationTokenSource cancellation)
    {
        var token = cancellation.Token;

        try
        {
            ConnectionInfo connectionInfo = new(user.IP, user.Username,
                new PasswordAuthenticationMethod(user.Username, user.Password))
            {
                Timeout = TimeSpan.FromSeconds(5)
            };

            using SshClient client = new(connectionInfo);
            await client.ConnectAsync(token).ConfigureAwait(false);

            using var command = client.CreateCommand(message);
            var execution = command.ExecuteAsync(token);
            await ReadOutput(command.OutputStream, execution, LogUpdate, token).ConfigureAwait(false);

            if (command.ExitStatus != 0)
            {
                var error = string.IsNullOrEmpty(command.Error) ? $"Exit status {command.ExitStatus}." : command.Error;
                LogUpdate($"Error: {error}");
            }
            else
            {
                LogUpdate("Program ended.");
            }
        }
        catch (OperationCanceledException) when (token.IsCancellationRequested)
        {
            LogUpdate("Program canceled.");
        }
        catch (Exception e)
        {
            LogUpdate($"Error: {e.Message}");
        }
        finally
        {
            lock (_sync)
            {
                _cancelToken = null;
                cancellation.Dispose();
            }
        }
    }

    internal static async Task ReadOutput(Stream output, Task execution, Action<string> log, CancellationToken cancellation)
    {
        using StreamReader reader = new(output);

        try
        {
            while (await reader.ReadLineAsync(cancellation).ConfigureAwait(false) is { } text)
                log(text);
        }
        finally
        {
            await execution.ConfigureAwait(false);
        }
    }

    void LogAdd(string text)
    {
        lock (Log)
            Log.Insert(0, $"{DateTime.Now:T} - {text}");
    }

    void LogUpdate(string text)
    {
        LogAdd(text);
        Update?.Invoke();
    }

    public void Dispose()
    {
        lock (_sync)
        {
            if (_disposed)
                return;

            _disposed = true;
            Update = null;
            _cancelToken?.Cancel();
        }

        GC.SuppressFinalize(this);
    }
}
