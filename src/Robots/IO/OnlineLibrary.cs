using System.Globalization;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;

namespace Robots;

class FileDto
{
    public required string Name { get; init; }
    public required string Sha { get; init; }
}

public class LibraryItem(string name)
{
    public string Name { get; } = name;
    public bool IsLocal { get; internal set; }
    internal string? OnlineSha { get; set; }
    internal string? DownloadedSha { get; set; }

    public bool IsOnline => OnlineSha is not null;
    public bool IsDownloaded => DownloadedSha is not null;
    public bool IsUpdateAvailable => IsOnline && (OnlineSha != DownloadedSha);
}

public class OnlineLibrary : IDisposable
{
    static readonly JsonSerializerOptions JsonOptions = new() { PropertyNamingPolicy = JsonNamingPolicy.CamelCase };

    readonly HttpClient _http = new();
    public Dictionary<string, LibraryItem> Libraries { get; } = new(StringComparer.OrdinalIgnoreCase);
    public event Action? LibraryChanged;

    public OnlineLibrary()
    {
        var headers = _http.DefaultRequestHeaders;
        headers.Add("Accept", "application/vnd.github.v3+json");
        headers.Add("User-Agent", "request");
    }

    public Task UpdateLibraryAsync() => UpdateLibraryAsync(_http, RobotLibrary.OnlinePath, RobotLibrary.LocalPath);

    internal async Task UpdateLibraryAsync(HttpClient http, string onlinePath, string localPath)
    {
        Libraries.Clear();
        AddDiskLibraries(onlinePath, false);
        AddDiskLibraries(localPath, true);
        await AddOnlineLibrariesAsync(http);
    }

    public async Task DownloadLibraryAsync(LibraryItem library)
    {
        await DownloadLibrary(library, _http, RobotLibrary.OnlinePath);
        LibraryChanged?.Invoke();
    }

    internal static async Task DownloadLibrary(LibraryItem library, HttpClient http, string folder)
    {
        if (!library.IsUpdateAvailable)
            throw new ArgumentException("Library does not require update.", nameof(library));

        ArgumentException.ThrowIfNullOrWhiteSpace(library.Name);

        if (Path.GetFileName(library.Name) != library.Name)
            throw new ArgumentException("Library name must be a file name.", nameof(library));

        var staging = Path.Combine(folder, $".download-{Guid.NewGuid():N}");
        _ = Directory.CreateDirectory(staging);
        string[] files = [library.Name + ".xml", library.Name + ".3dm"];
        int installed = 0;

        try
        {
            foreach (var file in files)
            {
                var uri = new Uri("https://raw.githubusercontent.com/visose/Robots/libraries/" + Uri.EscapeDataString(file));
                using var content = await http.GetStreamAsync(uri);
                using var output = File.Create(Path.Combine(staging, file));
                await content.CopyToAsync(output);
            }

            var sha = GetLocalSha(Path.Combine(staging, files[0]));

            if (sha != library.OnlineSha)
                throw new InvalidOperationException("Downloaded files do not match the online library.");

            foreach (var file in files)
            {
                var source = Path.Combine(staging, file);
                var destination = Path.Combine(folder, file);

                if (File.Exists(destination))
                    File.Replace(source, destination, source + ".bak");
                else
                    File.Move(source, destination);

                installed++;
            }

            library.DownloadedSha = sha;
        }
        catch
        {
            while (installed > 0)
            {
                var file = files[installed - 1];
                var backup = Path.Combine(staging, file + ".bak");
                var destination = Path.Combine(folder, file);

                if (File.Exists(backup))
                    File.Move(backup, destination, overwrite: true);
                else
                    File.Delete(destination);

                installed--;
            }

            throw;
        }
        finally
        {
            // Keep backups if rollback itself failed.
            if (installed == 0 || installed == files.Length)
                Directory.Delete(staging, recursive: true);
        }
    }

    public void RemoveDownloadedLibrary(LibraryItem item)
    {
        var folder = RobotLibrary.OnlinePath;
        string pathXml = Path.Combine(folder, item.Name + ".xml");
        string path3dm = Path.Combine(folder, item.Name + ".3dm");

        File.Delete(pathXml);
        File.Delete(path3dm);

        item.DownloadedSha = null;
        LibraryChanged?.Invoke();
    }

    async Task AddOnlineLibrariesAsync(HttpClient http)
    {
        var uri = new Uri("https://api.github.com/repos/visose/robots/contents?ref=libraries");
        var json = await http.GetStringAsync(uri);
        var files = JsonSerializer.Deserialize<List<FileDto>>(json, JsonOptions)
            ?? throw new InvalidOperationException("Could not list libraries.");

        foreach (var file in files)
        {
            var extension = GetValidExtension(file.Name);

            if (extension is null)
                continue;

            var name = Path.GetFileNameWithoutExtension(file.Name);

            if (!Libraries.TryGetValue(name, out var value))
            {
                value = new(name);
                Libraries.Add(name, value);
            }

            var sha = value.OnlineSha;
            value.OnlineSha = extension switch
            {
                ".xml" => file.Sha + sha,
                ".3dm" => sha + file.Sha,
                _ => throw new InvalidOperationException("Invalid library extension."),
            };
        }
    }

    static string? GetValidExtension(string fileName)
    {
        var extension = Path.GetExtension(fileName);
        return extension.EqualsIgnoreCase(".xml") ? ".xml" : extension.EqualsIgnoreCase(".3dm") ? ".3dm" : null;
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
                value = new(name);
                Libraries.Add(name, value);
            }

            if (isLocal)
            {
                value.IsLocal = true;
            }
            else
            {
                value.DownloadedSha = GetLocalSha(file);
            }
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
        using var stream = File.OpenRead(file);
        var header = Encoding.UTF8.GetBytes($"blob {stream.Length.ToString(CultureInfo.InvariantCulture)}\0");
#pragma warning disable CA5350 // GitHub content API uses Git blob SHA-1s; this is not used for security.
        using var hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA1);
#pragma warning restore CA5350
        hash.AppendData(header);
        Span<byte> buffer = stackalloc byte[8192];
        int count;

        while ((count = stream.Read(buffer)) > 0)
            hash.AppendData(buffer[..count]);

        return Convert.ToHexString(hash.GetHashAndReset()).ToLowerInvariant();
    }

    public void Dispose()
    {
        _http.Dispose();
        GC.SuppressFinalize(this);
    }
}
