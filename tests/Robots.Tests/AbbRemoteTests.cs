using System.Text.Json;
using NUnit.Framework;

namespace Robots.Tests;

public class AbbRemoteTests
{
    const string HelperPath = "Robots.AbbRemote.exe";
    static readonly TimeSpan TestTimeout = TimeSpan.FromSeconds(1);

    [Test]
    public void MissingHelperGivesClearError()
    {
        if (!OperatingSystem.IsWindows())
            Assert.Ignore("ABB remote helper launch tests require Windows.");

        string helperPath = Path.Combine(TestContext.CurrentContext.WorkDirectory, "missing", "Robots.AbbRemote.exe");
        RemoteAbb remote = new(helperPath, new AbbRemoteProcessRunner(), TestTimeout);

        var exception = Assert.Throws<FileNotFoundException>(remote.Play);

        Assert.That(exception!.Message, Does.Contain("ABB remote helper was not found"));
        Assert.That(remote.Log, Has.Some.Contains("Error: ABB remote helper was not found"));
    }

    [Test]
    public void MissingIpIsSentAsNull()
    {
        if (!OperatingSystem.IsWindows())
            Assert.Ignore("RemoteAbb command tests require Windows.");

        CapturingRunner runner = new();
        RemoteAbb remote = new(HelperPath, runner, TestTimeout);

        remote.Play();

        Assert.That(runner.Request?.Ip, Is.Null);
        Assert.That(remote.Log, Has.Some.Contains("first available controller"));
    }

    [Test]
    public void UploadRejectsMissingProgramCode()
    {
        MissingCodeProgram program = new(TestRobots.AbbIrb120());
        RemoteAbb remote = new(HelperPath, new CapturingRunner(), TestTimeout);

        var exception = Assert.Throws<InvalidOperationException>(() => remote.Upload(program));

        Assert.That(exception!.Message, Does.Contain("program code was not generated"));
    }

    [Test]
    public void FailedResponseThrowsClearErrorAndAddsHelperLog()
    {
        if (!OperatingSystem.IsWindows())
            Assert.Ignore("RemoteAbb command tests require Windows.");

        CapturingRunner runner = new()
        {
            Response = request => AbbRemoteResponse.Failure(
                request.Id,
                "Robot Communication Runtime is missing.",
                AbbRemoteErrorCodes.MissingRuntime,
                ["helper checked runtime"])
        };

        RemoteAbb remote = new(HelperPath, runner, TestTimeout);

        var exception = Assert.Throws<AbbRemoteException>(remote.Play);

        Assert.That(exception!.ErrorCode, Is.EqualTo(AbbRemoteErrorCodes.MissingRuntime));
        Assert.That(exception.Message, Is.EqualTo("Robot Communication Runtime is missing."));
        Assert.That(remote.Log, Has.Some.Contains("helper checked runtime"));
    }

    [Test]
    public void ProcessRunnerFailsFastOnBadHelperBehavior()
    {
        Assert.Multiple(() =>
        {
            FakeProcess timeoutProcess = new(waitForExit: false);
            var timeout = Assert.Throws<TimeoutException>(() => RunWithTempHelper(timeoutProcess));
            Assert.That(timeout!.Message, Does.Contain("timed out"));
            Assert.That(timeoutProcess.Killed, Is.True);

            var nonzero = Assert.Throws<InvalidOperationException>(() => RunWithTempHelper(new FakeProcess(exitCode: 7, stderr: "boom")));
            Assert.That(nonzero!.Message, Does.Contain("code 7"));
            Assert.That(nonzero.Message, Does.Contain("boom"));

            var stderr = Assert.Throws<InvalidOperationException>(() => RunWithTempHelper(new FakeProcess(stdout: SuccessJson("request-id"), stderr: "warning")));
            Assert.That(stderr!.Message, Does.Contain("wrote to stderr"));
            Assert.That(stderr.Message, Does.Contain("warning"));

            _ = Assert.Throws<JsonException>(() => RunWithTempHelper(new FakeProcess(stdout: "not json")));
        });
    }

    [Test]
    public void ProcessRunnerReturnsFailedResponse()
    {
        AbbRemoteResponse response = AbbRemoteResponse.Failure("request-id", "Controller unavailable.", AbbRemoteErrorCodes.ControllerUnavailable, []);
        FakeProcess process = new(stdout: AbbRemoteProtocol.SerializeResponse(response));
        AbbRemoteProcessRunner runner = new(new FakeProcessFactory(process));

        var actual = RunWithTempHelper(runner);

        Assert.That(actual.Ok, Is.False);
        Assert.That(actual.ErrorCode, Is.EqualTo(AbbRemoteErrorCodes.ControllerUnavailable));
    }

    [Test]
    public void JsonRequestResponseRoundTrips()
    {
        AbbRemoteRequest request = new("1", AbbRemoteActions.Upload, "192.168.0.10", "TestProgram", @"C:\Temp\TestProgram");
        AbbRemoteResponse response = AbbRemoteResponse.Failure("1", "No controller.", AbbRemoteErrorCodes.ControllerUnavailable, ["scan"]);

        var requestJson = AbbRemoteProtocol.SerializeRequest(request);
        var responseJson = AbbRemoteProtocol.SerializeResponse(response);

        var actualResponse = AbbRemoteProtocol.DeserializeResponse(responseJson);

        Assert.Multiple(() =>
        {
            Assert.That(AbbRemoteProtocol.DeserializeRequest(requestJson), Is.EqualTo(request));
            Assert.That(actualResponse.Id, Is.EqualTo(response.Id));
            Assert.That(actualResponse.Ok, Is.EqualTo(response.Ok));
            Assert.That(actualResponse.Message, Is.EqualTo(response.Message));
            Assert.That(actualResponse.Log, Is.EqualTo(response.Log));
            Assert.That(actualResponse.ErrorCode, Is.EqualTo(response.ErrorCode));
            Assert.That(requestJson, Does.Contain("programName"));
            Assert.That(responseJson, Does.Contain("errorCode"));
        });
    }

    static AbbRemoteResponse RunWithTempHelper(FakeProcess process)
    {
        AbbRemoteProcessRunner runner = new(new FakeProcessFactory(process));
        return RunWithTempHelper(runner);
    }

    static AbbRemoteResponse RunWithTempHelper(AbbRemoteProcessRunner runner)
    {
        string helperPath = Path.GetTempFileName();

        try
        {
            AbbRemoteRequest request = new("request-id", AbbRemoteActions.Play, null, null, null);
            return runner.Run(helperPath, request, TimeSpan.FromMilliseconds(10));
        }
        finally
        {
            File.Delete(helperPath);
        }
    }

    static string SuccessJson(string id) =>
        AbbRemoteProtocol.SerializeResponse(AbbRemoteResponse.Success(id, "OK", []));

    sealed class CapturingRunner : IAbbRemoteProcessRunner
    {
        public Func<AbbRemoteRequest, AbbRemoteResponse> Response { get; set; } =
            request => AbbRemoteResponse.Success(request.Id, "OK", []);

        public AbbRemoteRequest? Request { get; private set; }

        public AbbRemoteResponse Run(string helperPath, AbbRemoteRequest request, TimeSpan timeout)
        {
            Request = request;
            return Response(request);
        }
    }

    sealed class FakeProcessFactory(FakeProcess process) : IAbbRemoteProcessFactory
    {
        public IAbbRemoteProcess Start(string helperPath) => process;
    }

    sealed class FakeProcess(bool waitForExit = true, int exitCode = 0, string? stdout = null, string? stderr = null) : IAbbRemoteProcess
    {
        readonly bool _waitForExit = waitForExit;

        public TextWriter StandardInput { get; } = new StringWriter();
        public TextReader StandardOutput { get; } = new StringReader(stdout ?? SuccessJson("request-id"));
        public TextReader StandardError { get; } = new StringReader(stderr ?? "");
        public int ExitCode { get; } = exitCode;
        public bool Killed { get; private set; }

        public bool WaitForExit(int milliseconds) => _waitForExit;

        public void Kill()
        {
            Killed = true;
        }

        public void Dispose()
        { }
    }

    sealed class MissingCodeProgram(RobotSystem robotSystem) : IProgram
    {
        public string Name => "TestProgram";
        public RobotSystem RobotSystem { get; } = robotSystem;
        public List<List<List<string>>>? Code => null;
        public bool HasSimulation => false;
        public IReadOnlyList<int> MultiFileIndices { get; } = [0];

        public void Save(string folder)
        {
            throw new InvalidOperationException("Save should not be called.");
        }
    }
}
