namespace Robots;

class User
{
    public string IP { get; set; } = "";
    public string Username { get; set; } = "root";
    public string Password { get; set; } = "easybot";
    public string ProgramsDir { get; set; } = "/programs";

    public User(string? ip)
    {
        if (ip is null)
            throw new ArgumentNullException(nameof(ip));

        Uri uri = new(ip);
        IP = uri.Host;

        if (!string.IsNullOrWhiteSpace(uri.UserInfo))
        {
            var split = uri.UserInfo.Split(':');

            if (split is not null && split.Length == 2)
            {
                Username = split[0];
                Password = split[1];
            }
        }

        if (!string.IsNullOrWhiteSpace(uri.PathAndQuery) && uri.PathAndQuery != "/")
            ProgramsDir = uri.PathAndQuery;
    }

    public void Deconstruct(out string ip, out string username, out string password, out string programsDir)
    {
        ip = IP;
        username = Username;
        password = Password;
        programsDir = ProgramsDir;
    }
}
