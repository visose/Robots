using Renci.SshNet;
using System.Text;

namespace Robots;

public class RemoteFranka : IRemote
{
    User? _user;
    string? _uploadedFile;

    public Action? Update { get; set; }
    public List<string> Log { get; } = new();

    CancellationTokenSource? _cancelToken;
    Task? _task;

    public string? IP
    {
        get => _user?.IP;
        set
        {
            try
            {
                _user = new(value);
            }
            catch
            {
                _user = null;
                LogAdd($"Invalid address: {value}");
            }
        }
    }

    public void Upload(IProgram program)
    {
        if (_user is null)
        {
            LogAdd($"Error: IP not set.");
            return;
        }

        if (program.Code is null)
        {
            LogAdd($"Error: Program code not generated.");
            return;
        }

        try
        {
            var code = string.Join("\n", program.Code[0].SelectMany(c => c));
            var bytes = Encoding.ASCII.GetBytes(code);
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
        if (_cancelToken is null)
        {
            LogAdd("Program not running.");
            return;
        }

        _cancelToken?.Cancel();
    }

    public void Play()
    {
        if (_user is null)
        {
            LogAdd($"Error: IP not set.");
            return;
        }

        if (_uploadedFile is null)
        {
            LogAdd($"Error: File not uploaded.");
            return;
        }

        if (_task?.IsCompleted == false)
        {
            return;
        }

        Send($"python -u {_user.ProgramsDir}/{_uploadedFile}");
    }

    public void Send(string message)
    {
        try
        {
            SendPrivate(message);
        }
        catch (Exception e)
        {
            LogAdd($"Error: {e.Message}");
        }
    }

    void SendPrivate(string message)
    {
        if (_user is null)
            throw new ArgumentNullException("IP not set.");

        var (ip, username, password, _) = _user;

        ConnectionInfo connectionInfo = new(ip, username,
            new PasswordAuthenticationMethod(username, password))
        {
            Timeout = TimeSpan.FromSeconds(5)
        };

        _cancelToken = new();
        var token = _cancelToken.Token;

        _task = Task.Run(async () =>
        {
            using SshClient client = new(connectionInfo);
            client.Connect();

            using var command = client.CreateCommand(message);
            var async = command.BeginExecute();

            var reader = new StreamReader(command.OutputStream);

            while (!async.IsCompleted)
            {
                await Task.Delay(100);

                if (token.IsCancellationRequested)
                    command.CancelAsync();

                string? text;

                while ((text = await reader.ReadLineAsync()) is not null)
                    LogUpdate(text);
            }

            var end = reader.ReadToEnd();

            if (!string.IsNullOrEmpty(end))
                LogUpdate(end);

            if (command.ExitStatus != 0)
            {
                var error = string.IsNullOrEmpty(command.Error)
                ? "Program canceled."
                : command.Error;

                LogUpdate($"Error: {error}");
            }
            else
            {
                LogUpdate("Program ended.");
            }

            command.EndExecute(async);
        }, token)
            .ContinueWith(t =>
            {
                _cancelToken.Dispose();
                _cancelToken = null;
            });
    }

    void LogAdd(string text) =>
        Log.Insert(0, $"{DateTime.Now.ToLongTimeString()} - {text}");

    void LogUpdate(string text)
    {
        LogAdd(text);
        Update?.Invoke();
    }
}
