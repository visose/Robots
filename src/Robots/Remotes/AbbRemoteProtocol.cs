using System.Text.Json;
using System.Text.Json.Serialization;

namespace Robots;

internal static class AbbRemoteActions
{
    public const string Upload = "upload";
    public const string Play = "play";
    public const string Pause = "pause";

    public static bool IsKnown(string action) =>
        action is Upload or Play or Pause;
}

internal static class AbbRemoteErrorCodes
{
    public const string MissingRuntime = "MissingRuntime";
    public const string ConnectionFailed = "ConnectionFailed";
    public const string ControllerUnavailable = "ControllerUnavailable";
    public const string PermissionDenied = "PermissionDenied";
    public const string MastershipFailed = "MastershipFailed";
    public const string InvalidState = "InvalidState";
    public const string UploadFailed = "UploadFailed";
    public const string LoadFailed = "LoadFailed";
    public const string Unsupported = "Unsupported";
}

internal sealed record AbbRemoteRequest(
    string Id,
    string Action,
    string? Ip,
    string? ProgramName,
    string? LocalFolder);

internal sealed record AbbRemoteResponse(
    string Id,
    bool Ok,
    string Message,
    string[] Log,
    string? ErrorCode)
{
    public static AbbRemoteResponse Success(string id, string message, IReadOnlyCollection<string> log) =>
        new(id, true, message, [.. log], null);

    public static AbbRemoteResponse Failure(string id, string message, string errorCode, IReadOnlyCollection<string> log) =>
        new(id, false, message, [.. log], errorCode);
}

internal static class AbbRemoteProtocol
{
    static readonly JsonSerializerOptions Options = new()
    {
        DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull,
        PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
        PropertyNameCaseInsensitive = true,
    };

    public static string SerializeRequest(AbbRemoteRequest request)
    {
        ValidateRequest(request);
        return JsonSerializer.Serialize(request, Options);
    }

    public static AbbRemoteRequest DeserializeRequest(string json)
    {
        var request = JsonSerializer.Deserialize<AbbRemoteRequest>(json, Options)
            ?? throw new JsonException("ABB remote request JSON was empty.");

        ValidateRequest(request);
        return request;
    }

    public static string SerializeResponse(AbbRemoteResponse response)
    {
        ValidateResponse(response);
        return JsonSerializer.Serialize(response, Options);
    }

    public static AbbRemoteResponse DeserializeResponse(string json)
    {
        var response = JsonSerializer.Deserialize<AbbRemoteResponse>(json, Options)
            ?? throw new JsonException("ABB remote response JSON was empty.");

        ValidateResponse(response);
        return response;
    }

    static void ValidateRequest(AbbRemoteRequest request)
    {
        ArgumentNullException.ThrowIfNull(request);

        if (string.IsNullOrWhiteSpace(request.Id))
            throw new JsonException("ABB remote request is missing id.");

        if (string.IsNullOrWhiteSpace(request.Action))
            throw new JsonException("ABB remote request is missing action.");

        if (!AbbRemoteActions.IsKnown(request.Action))
            throw new JsonException($"ABB remote request action is unsupported: {request.Action}");

        if (request.Action == AbbRemoteActions.Upload)
        {
            if (string.IsNullOrWhiteSpace(request.ProgramName))
                throw new JsonException("ABB remote upload request is missing programName.");

            if (string.IsNullOrWhiteSpace(request.LocalFolder))
                throw new JsonException("ABB remote upload request is missing localFolder.");
        }
    }

    static void ValidateResponse(AbbRemoteResponse response)
    {
        ArgumentNullException.ThrowIfNull(response);

        if (string.IsNullOrWhiteSpace(response.Id))
            throw new JsonException("ABB remote response is missing id.");

        if (string.IsNullOrWhiteSpace(response.Message))
            throw new JsonException("ABB remote response is missing message.");

        if (response.Log is null)
            throw new JsonException("ABB remote response is missing log.");

        if (!response.Ok && string.IsNullOrWhiteSpace(response.ErrorCode))
            throw new JsonException("ABB remote failed response is missing errorCode.");
    }
}
