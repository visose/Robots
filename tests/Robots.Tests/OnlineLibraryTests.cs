using System.Net;
using System.Security.Cryptography;
using System.Text;
using NUnit.Framework;

namespace Robots.Tests;

class OnlineLibraryTests
{
    string _folder = null!;
    static readonly byte[] Xml = Encoding.UTF8.GetBytes("<RobotSystem />");
    static readonly byte[] Mesh = [.. Enumerable.Range(0, 25000).Select(i => (byte)i)];

    [SetUp]
    public void CreateFolder() => _folder = Directory.CreateTempSubdirectory("robots-library-tests-").FullName;

    [TearDown]
    public void RemoveFolder() => Directory.Delete(_folder, recursive: true);

    [TestCase(false)]
    [TestCase(true)]
    public async Task RefreshKeepsDiskLibrariesWhenOffline(bool online)
    {
        var downloaded = Directory.CreateDirectory(Path.Combine(_folder, "downloaded")).FullName;
        var local = Directory.CreateDirectory(Path.Combine(_folder, "local")).FullName;
        File.WriteAllBytes(Path.Combine(downloaded, "Test.xml"), Xml);
        File.WriteAllBytes(Path.Combine(downloaded, "Test.3dm"), Mesh);
        File.WriteAllBytes(Path.Combine(local, "Local.xml"), Xml);
        using OnlineLibrary library = new();
        using HttpClient http = new(new CatalogueHandler(online));

        if (online)
            await library.UpdateLibraryAsync(http, downloaded, local);
        else
            Assert.That(async () => await library.UpdateLibraryAsync(http, downloaded, local), Throws.TypeOf<HttpRequestException>());

        Assert.Multiple(() =>
        {
            Assert.That(library.Libraries.Keys, Is.EquivalentTo((string[])["Test", "Local"]));
            Assert.That(library.Libraries["Test"].IsDownloaded, Is.True);
            Assert.That(library.Libraries["Test"].IsOnline, Is.EqualTo(online));
            Assert.That(library.Libraries["Test"].IsUpdateAvailable, Is.False);
            Assert.That(library.Libraries["Local"].IsLocal, Is.True);
        });
    }

    [TestCase(false)]
    [TestCase(true)]
    public async Task DownloadInstallsVerifiedPair(bool update)
    {
        var library = CreateLibrary(update);
        using HttpClient http = new(new DownloadHandler());

        await OnlineLibrary.DownloadLibrary(library, http, _folder);

        Assert.Multiple(() =>
        {
            Assert.That(File.ReadAllBytes(Path.Combine(_folder, "Test.xml")), Is.EqualTo(Xml));
            Assert.That(File.ReadAllBytes(Path.Combine(_folder, "Test.3dm")), Is.EqualTo(Mesh));
            Assert.That(library.DownloadedSha, Is.EqualTo(library.OnlineSha));
            Assert.That(library.IsUpdateAvailable, Is.False);
            Assert.That(Directory.GetDirectories(_folder), Is.Empty);
        });
    }

    [TestCase(false)]
    [TestCase(true)]
    public void FailedDownloadPreservesInstalledPair(bool corrupt)
    {
        var library = CreateLibrary(update: true);
        using HttpClient http = new(new DownloadHandler(corrupt ? HttpStatusCode.OK : HttpStatusCode.NotFound, corrupt));

        Assert.That(async () => await OnlineLibrary.DownloadLibrary(library, http, _folder),
            corrupt ? Throws.TypeOf<InvalidOperationException>() : Throws.TypeOf<HttpRequestException>());

        Assert.Multiple(() =>
        {
            Assert.That(File.ReadAllText(Path.Combine(_folder, "Test.xml")), Is.EqualTo("old XML"));
            Assert.That(File.ReadAllText(Path.Combine(_folder, "Test.3dm")), Is.EqualTo("old mesh"));
            Assert.That(library.DownloadedSha, Is.EqualTo("old"));
            Assert.That(Directory.GetDirectories(_folder), Is.Empty);
        });
    }

    [TestCase(false)]
    [TestCase(true)]
    public void FailedInstallRollsBackFirstFile(bool update)
    {
        var library = CreateLibrary(update);
        var meshPath = Path.Combine(_folder, "Test.3dm");
        File.Delete(meshPath);
        _ = Directory.CreateDirectory(meshPath);
        using HttpClient http = new(new DownloadHandler());

        Assert.That(async () => await OnlineLibrary.DownloadLibrary(library, http, _folder), Throws.TypeOf<IOException>());

        var xmlPath = Path.Combine(_folder, "Test.xml");

        Assert.Multiple(() =>
        {
            Assert.That(File.Exists(xmlPath), Is.EqualTo(update));

            if (update)
                Assert.That(File.ReadAllText(xmlPath), Is.EqualTo("old XML"));

            Assert.That(library.DownloadedSha, Is.EqualTo(update ? "old" : null));
            Assert.That(Directory.GetDirectories(_folder), Is.EqualTo(new[] { meshPath }));
        });
    }

    LibraryItem CreateLibrary(bool update)
    {
        if (update)
        {
            File.WriteAllText(Path.Combine(_folder, "Test.xml"), "old XML");
            File.WriteAllText(Path.Combine(_folder, "Test.3dm"), "old mesh");
        }

        return new("Test") { OnlineSha = BlobSha(Xml) + BlobSha(Mesh), DownloadedSha = update ? "old" : null };
    }

    static string BlobSha(byte[] content)
    {
        byte[] blob = [.. Encoding.UTF8.GetBytes($"blob {content.Length}\0"), .. content];
#pragma warning disable CA5350 // Git blob checksum, not a security primitive.
        return Convert.ToHexString(SHA1.HashData(blob)).ToLowerInvariant();
#pragma warning restore CA5350
    }

    class CatalogueHandler(bool online) : HttpMessageHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken) =>
            Task.FromResult(new HttpResponseMessage(online ? HttpStatusCode.OK : HttpStatusCode.ServiceUnavailable)
            {
                Content = new StringContent($$"""[{"name":"Test.xml","sha":"{{BlobSha(Xml)}}"},{"name":"Test.3dm","sha":"{{BlobSha(Mesh)}}"}]""")
            });
    }

    class DownloadHandler(HttpStatusCode meshStatus = HttpStatusCode.OK, bool corrupt = false) : HttpMessageHandler
    {
        protected override Task<HttpResponseMessage> SendAsync(HttpRequestMessage request, CancellationToken cancellationToken)
        {
            bool xml = request.RequestUri!.AbsolutePath.EndsWith(".xml", StringComparison.Ordinal);
            return Task.FromResult(new HttpResponseMessage(xml ? HttpStatusCode.OK : meshStatus)
            {
                Content = new ByteArrayContent(xml ? Xml : corrupt ? [0] : Mesh)
            });
        }
    }
}
