using System.Text;
using NUnit.Framework;
using Renci.SshNet.Common;

namespace Robots.Tests;

public class RemoteFrankaTests
{
    [Test]
    public async Task QuietCommandCanBeCanceled()
    {
        using CancellationTokenSource cancellation = new();
        using PipeStream output = new();
        using var registration = cancellation.Token.Register(output.Dispose);
        var execution = Task.Delay(Timeout.Infinite, cancellation.Token);
        var reading = RemoteFranka.ReadOutput(output, execution, _ => Assert.Fail("Command produced no output."), cancellation.Token);

        await cancellation.CancelAsync();

        _ = Assert.ThrowsAsync<TaskCanceledException>(async () => await reading.WaitAsync(TimeSpan.FromSeconds(5)));
    }

    [Test]
    public async Task ReadsFinalLineWithoutNewline()
    {
        using MemoryStream output = new(Encoding.UTF8.GetBytes("first\nlast"));
        List<string> lines = [];
        string[] expected = ["first", "last"];

        await RemoteFranka.ReadOutput(output, Task.CompletedTask, lines.Add, CancellationToken.None);

        Assert.That(lines, Is.EqualTo(expected));
    }

    [Test]
    public void ReportsExecutionFailureAfterOutputEnds()
    {
        using MemoryStream output = new();
        InvalidOperationException failure = new("SSH connection failed.");

        var exception = Assert.ThrowsAsync<InvalidOperationException>(() =>
            RemoteFranka.ReadOutput(output, Task.FromException(failure), _ => { }, CancellationToken.None));

        Assert.That(exception, Is.SameAs(failure));
    }
}
