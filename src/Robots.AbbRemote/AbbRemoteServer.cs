using System.Text.Json;

using Robots;

namespace Robots.AbbRemote;

internal sealed class AbbRemoteServer(PcSdkAbbClient client)
{
    public async Task<int> RunAsync(TextReader input, TextWriter output)
    {
        string id = "unknown";
        AbbRemoteResponse response;

        try
        {
            string? line = await input.ReadLineAsync();

            if (string.IsNullOrWhiteSpace(line))
                throw new AbbRemoteHelperException("ABB remote helper did not receive a request.", AbbRemoteErrorCodes.Unsupported);

            AbbRemoteRequest request = AbbRemoteProtocol.DeserializeRequest(line);
            id = request.Id;
            response = Dispatch(request);
        }
        catch (JsonException exception)
        {
            response = AbbRemoteResponse.Failure(id, $"Invalid ABB remote request: {exception.Message}", AbbRemoteErrorCodes.Unsupported, []);
        }
        catch (AbbRemoteHelperException exception)
        {
            response = AbbRemoteResponse.Failure(id, exception.Message, exception.ErrorCode, exception.Log);
        }
        catch (Exception exception)
        {
            response = AbbRemoteResponse.Failure(id, exception.Message, AbbRemoteErrorCodes.Unsupported, []);
        }

        string json = AbbRemoteProtocol.SerializeResponse(response);
        await output.WriteLineAsync(json);
        return 0;
    }

    AbbRemoteResponse Dispatch(AbbRemoteRequest request) =>
        request.Action switch
        {
            AbbRemoteActions.Upload => client.Upload(request),
            AbbRemoteActions.Play => client.Play(request),
            AbbRemoteActions.Pause => client.Pause(request),
            _ => throw new AbbRemoteHelperException($"Unsupported ABB remote action: {request.Action}", AbbRemoteErrorCodes.Unsupported),
        };

}

internal sealed class AbbRemoteHelperException(
    string message,
    string errorCode,
    IReadOnlyCollection<string>? log = null,
    Exception? innerException = null)
    : InvalidOperationException(message, innerException)
{
    public string ErrorCode { get; } = errorCode;
    public IReadOnlyCollection<string> Log { get; } = log ?? [];
}
