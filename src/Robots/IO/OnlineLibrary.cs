using Octokit;
using System.Security.Cryptography;

namespace Robots;

public class LibraryItem
{
    public string Name { get; set; } = default!;
    public bool IsLocal { get; set; }
    internal string? OnlineSha { get; set; }
    internal string? DownloadedSha { get; set; }

    public bool IsOnline => OnlineSha is not null;
    public bool IsDownloaded => DownloadedSha is not null;
    public bool IsUpdateAvailable => IsOnline && (OnlineSha != DownloadedSha);
}

public class OnlineLibrary
{
    readonly HttpClient _http = new();
    public Dictionary<string, LibraryItem> Libraries { get; } = new();
    public event Action? LibraryChanged;

    public OnlineLibrary()
    {
        _http.BaseAddress = new Uri("https://raw.githubusercontent.com/visose/Robots/master/libraries/");
    }

    public async Task UpdateLibraryAsync()
    {
        Libraries.Clear();
        await AddOnlineLibrariesAsync();
        AddDiskLibraries(FileIO.OnlineLibraryPath, false);
        AddDiskLibraries(FileIO.LocalLibraryPath, true);
    }

    public async Task<bool> TryDownloadLibraryAsync(LibraryItem library)
    {
        if (!library.IsUpdateAvailable)
            throw new ArgumentException("Library does not require update.");

        if (!await TryDownloadFileAsync(library.Name + ".xml"))
            return false;

        if (!await TryDownloadFileAsync(library.Name + ".3dm"))
            return false;

        var xmlPath = Path.Combine(FileIO.OnlineLibraryPath, library.Name + ".xml");
        var sha = GetLocalSha(xmlPath);

        if (sha != library.OnlineSha)
            return false;

        library.DownloadedSha = sha;
        LibraryChanged?.Invoke();
        return true;
    }

    public bool TryRemoveDownloadedLibrary(LibraryItem item)
    {
        var folder = FileIO.OnlineLibraryPath;
        string pathXml = Path.Combine(folder, item.Name + ".xml");
        string path3dm = Path.Combine(folder, item.Name + ".3dm");

        try
        {
            File.Delete(pathXml);
            File.Delete(path3dm);
        }
        catch (Exception)
        {
            return false;
        }

        item.DownloadedSha = null;
        LibraryChanged?.Invoke();
        return true;
    }

    async Task AddOnlineLibrariesAsync()
    {
        var github = new GitHubClient(new ProductHeaderValue("visoseRobots"));
        var files = await github.Repository.Content.GetAllContents("visose", "Robots", "libraries");

        foreach (var file in files)
        {
            var extension = GetValidExtension(file.Name);

            if (extension is null)
                continue;

            var name = Path.GetFileNameWithoutExtension(file.Name);
            var key = name.ToLowerInvariant();

            if (!Libraries.TryGetValue(key, out var value))
            {
                value = new LibraryItem { Name = name };
                Libraries.Add(key, value);
            }

            var sha = value.OnlineSha;
            value.OnlineSha = extension switch
            {
                ".xml" => file.Sha + sha,
                ".3dm" => sha + file.Sha,
                _ => throw new ArgumentException("Invalid extension."),
            };
        }
    }

    string? GetValidExtension(string fileName)
    {
        foreach (var extension in new[] { ".xml", ".3dm" })
        {
            if (Path.GetExtension(fileName).Equals(extension, StringComparison.InvariantCultureIgnoreCase))
                return extension;
        }

        return null;
    }

    void AddDiskLibraries(string folder, bool isLocal)
    {
        if (!Directory.Exists(folder))
            return;

        var files = Directory.EnumerateFiles(folder, "*.xml");

        foreach (var file in files)
        {
            var name = Path.GetFileNameWithoutExtension(file);
            var key = name.ToLowerInvariant();

            if (!Libraries.TryGetValue(key, out var value))
            {
                value = new LibraryItem { Name = name };
                Libraries.Add(key, value);
            }

            if (isLocal)
                value.IsLocal = true;
            else
                value.DownloadedSha = GetLocalSha(file);
        }
    }

    string GetLocalSha(string xmlPath)
    {
        var shaXml = GetSha1(xmlPath);
        var sha3dm = GetSha1(Path.ChangeExtension(xmlPath, ".3dm"));
        return shaXml + sha3dm;
    }

    string GetSha1(string file)
    {
        var bytes = File.ReadAllBytes(file);
        return GetSha1(bytes);
    }

    string GetSha1(byte[] contentBytes)
    {
        var header = $"blob {contentBytes.Length}\0";
        var encoding = new System.Text.UTF8Encoding();
        var headerBytes = encoding.GetBytes(header);

        var bytes = Util.Combine(headerBytes, contentBytes);

        var sha1 = SHA1.Create();
        var hash = sha1.ComputeHash(bytes);
        var hashText = string.Concat(hash.Select(b => b.ToString("x2")));
        return hashText;
    }

    async Task<bool> TryDownloadFileAsync(string fileName)
    {
        var folder = FileIO.OnlineLibraryPath;

        if (!Directory.Exists(folder))
            Directory.CreateDirectory(folder);

        string downloadPath = Path.Combine(folder, fileName);
        var response = await _http.GetAsync(fileName);

        if (response.StatusCode != System.Net.HttpStatusCode.OK)
            return false;

        var bytes = await response.Content.ReadAsByteArrayAsync();
        File.WriteAllBytes(downloadPath, bytes);

        return true;
    }
}