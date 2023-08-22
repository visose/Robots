using System.Security.Cryptography;
using Newtonsoft.Json;

namespace Robots;

class FileDto
{
    public string Name { get; set; } = default!;
    public string Sha { get; set; } = default!;
}

public class LibraryItem
{
    public string Name { get; }
    public bool IsLocal { get; internal set; }
    internal string? OnlineSha { get; set; }
    internal string? DownloadedSha { get; set; }

    public bool IsOnline => OnlineSha is not null;
    public bool IsDownloaded => DownloadedSha is not null;
    public bool IsUpdateAvailable => IsOnline && (OnlineSha != DownloadedSha);

    public LibraryItem(string name)
    {
        Name = name;
    }
}

public class OnlineLibrary
{
    readonly HttpClient _http = new();
    public Dictionary<string, LibraryItem> Libraries { get; } = new(StringComparer.OrdinalIgnoreCase);
    public event Action? LibraryChanged;

    public OnlineLibrary()
    {
        var headers = _http.DefaultRequestHeaders;
        headers.Add("Accept", "application/vnd.github.v3+json");
        headers.Add("User-Agent", "request");
    }

    public async Task UpdateLibraryAsync()
    {
        Libraries.Clear();
        await AddOnlineLibrariesAsync();
        AddDiskLibraries(FileIO.OnlineLibraryPath, false);
        AddDiskLibraries(FileIO.LocalLibraryPath, true);
    }

    public async Task DownloadLibraryAsync(LibraryItem library)
    {
        if (!library.IsUpdateAvailable)
            throw new ArgumentException("Library does not require update.", nameof(library));

        await DownloadFileAsync(library.Name + ".xml");
        await DownloadFileAsync(library.Name + ".3dm");

        var xmlPath = Path.Combine(FileIO.OnlineLibraryPath, library.Name + ".xml");
        var sha = GetLocalSha(xmlPath);

        if (sha != library.OnlineSha)
            throw new InvalidOperationException(" Downloaded file does not match on line file.");

        library.DownloadedSha = sha;
        LibraryChanged?.Invoke();
    }

    public void RemoveDownloadedLibrary(LibraryItem item)
    {
        var folder = FileIO.OnlineLibraryPath;
        string pathXml = Path.Combine(folder, item.Name + ".xml");
        string path3dm = Path.Combine(folder, item.Name + ".3dm");

        File.Delete(pathXml);
        File.Delete(path3dm);

        item.DownloadedSha = null;
        LibraryChanged?.Invoke();
    }

    async Task AddOnlineLibrariesAsync()
    {
        var uri = new Uri("https://api.github.com/repos/visose/robots/contents?ref=libraries");
        var json = await _http.GetStringAsync(uri);
        var files = JsonConvert.DeserializeObject<List<FileDto>>(json)
            ?? throw new ArgumentNullException(" Could not list libraries.");

        foreach (var file in files)
        {
            var extension = GetValidExtension(file.Name);

            if (extension is null)
                continue;

            var name = Path.GetFileNameWithoutExtension(file.Name);

            if (!Libraries.TryGetValue(name, out var value))
            {
                value = new LibraryItem(name);
                Libraries.Add(name, value);
            }

            var sha = value.OnlineSha;
            value.OnlineSha = extension switch
            {
                ".xml" => file.Sha + sha,
                ".3dm" => sha + file.Sha,
                _ => throw new ArgumentException(" Invalid extension.", nameof(extension)),
            };
        }
    }

    static string? GetValidExtension(string fileName)
    {
        foreach (var extension in new[] { ".xml", ".3dm" })
        {
            if (Path.GetExtension(fileName).EqualsIgnoreCase(extension))
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

            if (!Libraries.TryGetValue(name, out var value))
            {
                value = new LibraryItem(name);
                Libraries.Add(name, value);
            }

            if (isLocal)
                value.IsLocal = true;
            else
                value.DownloadedSha = GetLocalSha(file);
        }
    }

    static string GetLocalSha(string xmlPath)
    {
        var shaXml = GetSha1(xmlPath);
        var sha3dm = GetSha1(Path.ChangeExtension(xmlPath, ".3dm"));
        return shaXml + sha3dm;
    }

    static string GetSha1(string file)
    {
        var bytes = File.ReadAllBytes(file);
        return GetSha1(bytes);
    }

    static string GetSha1(byte[] contentBytes)
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

    async Task DownloadFileAsync(string fileName)
    {
        var folder = FileIO.OnlineLibraryPath;

        if (!Directory.Exists(folder))
            Directory.CreateDirectory(folder);

        string downloadPath = Path.Combine(folder, fileName);

        var baseUri = new Uri("https://raw.githubusercontent.com/visose/Robots/libraries/");
        var uri = new Uri(baseUri, fileName);

        var bytes = await _http.GetByteArrayAsync(uri);
        File.WriteAllBytes(downloadPath, bytes);
    }
}
