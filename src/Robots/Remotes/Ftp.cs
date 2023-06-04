using Renci.SshNet;

namespace Robots;

class Ftp
{
    public static void Upload(byte[] bytes, string fileName, User user)
    {
        var (ip, username, password, programsDir) = user;

        ConnectionInfo connectionInfo = new(ip, username, new PasswordAuthenticationMethod(username, password))
        {
            Timeout = TimeSpan.FromSeconds(5)
        };

        // Remove ecdsa algorithms not supported in mono (MacOS)
        // https://github.com/sshnet/SSH.NET/issues/807
        var algsToRemove = connectionInfo.HostKeyAlgorithms.Keys
            .Where(algName => algName.StartsWith("ecdsa"))
            .ToList();

        foreach (var algName in algsToRemove)
            connectionInfo.HostKeyAlgorithms.Remove(algName);

        using SftpClient client = new(connectionInfo);
        client.Connect();

        if (!client.Exists(programsDir))
            throw new DirectoryNotFoundException($"\"{programsDir}\" folder not found.");

        using MemoryStream stream = new(bytes);
        string filePath = $"{programsDir}/{fileName}";
        client.UploadFile(stream, filePath, true);
        client.Disconnect();
    }
}
