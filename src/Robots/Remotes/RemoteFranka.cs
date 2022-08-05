using Renci.SshNet;
using System.Text;

namespace Robots;

public class RemoteFranka : IRemote
{
    User? _user;
    string? _uploadedFile;

    public Action? Update { get; set; }
    public List<string> Log { get; } = new();

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
                AddLog($"Invalid address: {value}");
            }
        }
    }

    public void Upload(IProgram program)
    {
        if (_user is null)
        {
            AddLog($"Error: IP not set.");
            return;
        }

        if (program.Code is null)
        {
            AddLog($"Error: Program code not generated.");
            return;
        }

        AddLog("Uploading program...");

        try
        {
            var code = string.Join("\n", program.Code[0].SelectMany(c => c));
            var bytes = Encoding.ASCII.GetBytes(code);
            string fileName = $"{program.Name}.py";
            Ftp.Upload(bytes, fileName, _user);
            _uploadedFile = fileName;
        }
        catch (Exception e)
        {
            AddLog($"Error: FTP - {e.Message}");
        }
    }

    public void Pause() => AddLog("Pause is not supported.");

    public void Play()
    {
        if (_user is null)
        {
            AddLog($"Error: IP not set.");
            return;
        }

        if (_uploadedFile is null)
        {
            AddLog($"Error: File not uploaded.");
            return;
        }

        AddLog("Starting program...");
        Send($"python {_user.ProgramsDir}/{_uploadedFile}");
    }

    public void Send(string message)
    {
        try
        {
            SendPrivate(message);
        }
        catch (Exception e)
        {
            AddLog($"Error: {e.Message}");
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

        _ = Task.Run(() =>
        {
            using SshClient client = new(connectionInfo);
            client.Connect();
            var result = client.RunCommand(message);

            if (result.ExitStatus == 0)
            {
                if (!string.IsNullOrEmpty(result.Result))
                    AddLog(result.Result);
            }
            else
            {
                AddLog($"Error: {result.Error}");
            }

            Update?.Invoke();
        });
    }

    void AddLog(string message) => Log.Add(message);
}
