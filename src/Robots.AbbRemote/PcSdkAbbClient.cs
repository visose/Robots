using System.Collections;
using System.Reflection;
using System.Runtime.Loader;

namespace Robots.AbbRemote;

internal sealed class PcSdkAbbClient
{
    static readonly Version RobotWare8 = new(8, 0);

    readonly List<string> _log = [];
    AbbPcSdk? _sdk;
    object? _controller;

    AbbPcSdk Sdk => _sdk ?? throw new InvalidOperationException("ABB PC SDK was not loaded.");

    public AbbRemoteResponse Upload(AbbRemoteRequest request) =>
        Execute(request, controller => Upload(controller, request));

    public AbbRemoteResponse Play(AbbRemoteRequest request) =>
        Execute(request, Play);

    public AbbRemoteResponse Pause(AbbRemoteRequest request) =>
        Execute(request, Pause);

    AbbRemoteResponse Execute(AbbRemoteRequest request, Func<object, string> command)
    {
        _log.Clear();

        try
        {
            Connect(request.Ip);

            if (_controller is null || !Get<bool>(_controller, "Connected"))
                throw new AbbRemoteHelperException("Could not connect to ABB controller.", AbbRemoteErrorCodes.ConnectionFailed, _log);

            string message = command(_controller);
            return AbbRemoteResponse.Success(request.Id, message, _log);
        }
        catch (AbbPcSdkLoadException exception)
        {
            return AbbRemoteResponse.Failure(request.Id, exception.Message, AbbRemoteErrorCodes.MissingRuntime, _log);
        }
        catch (AbbRemoteHelperException exception)
        {
            return AbbRemoteResponse.Failure(request.Id, exception.Message, exception.ErrorCode, exception.Log);
        }
        catch (Exception exception)
        {
            Exception failure = Unwrap(exception);
            return AbbRemoteResponse.Failure(request.Id, failure.Message, ErrorCodeFor(failure), _log);
        }
        finally
        {
            Disconnect();
        }
    }

    void Connect(string? ip)
    {
        Disconnect();
        _sdk = AbbPcSdk.Load();
        AbbPcSdk sdk = Sdk;
        AddLog($"Using ABB PC SDK from {sdk.Directory}.");
        AddLog("Scanning ABB controllers.");

        object scanner = sdk.Create("ABB.Robotics.Controllers.Discovery.NetworkScanner");
        _ = sdk.Invoke(scanner, "Scan");

        object? controllerInfo = Controllers(scanner)
            .FirstOrDefault(controller => ip is null || Get<object>(controller, "IPAddress").ToString() == ip);

        if (controllerInfo is null)
        {
            string message = ip is null
                ? "No ABB controller was found."
                : $"ABB controller was not found at {ip}.";

            throw new AbbRemoteHelperException(message, AbbRemoteErrorCodes.ControllerUnavailable, _log);
        }

        if (!string.Equals(Get<object>(controllerInfo, "Availability").ToString(), "Available", StringComparison.Ordinal))
            throw new AbbRemoteHelperException("ABB controller is not available.", AbbRemoteErrorCodes.ControllerUnavailable, _log);

        _controller = sdk.InvokeStatic("ABB.Robotics.Controllers.ControllerFactory", "CreateFrom", controllerInfo);
        _ = sdk.Invoke(_controller, "Logon", sdk.StaticValue("ABB.Robotics.Controllers.UserInfo", "DefaultUser"));

        if (Get<Version>(_controller, "RobotWareVersion") >= RobotWare8)
        {
            object controlStation = Get<object>(_controller, "ControlStation");
            _ = sdk.Invoke(controlStation, "Register", "Robots ABB Remote", Guid.NewGuid().ToString("N"), 1234u, true);
            AddLog("Registered ABB control station.");
        }

        AddLog($"Connected to ABB controller {Get<string>(_controller, "Name")}.");
    }

    void Disconnect()
    {
        if (_controller is null || _sdk is null)
            return;

        try
        {
            _ = _sdk.Invoke(_controller, "Logoff");
        }
        finally
        {
            (_controller as IDisposable)?.Dispose();
            _controller = null;
        }
    }

    string Upload(object controller, AbbRemoteRequest request)
    {
        string programName = Require(request.ProgramName, "programName");
        string localFolder = Require(request.LocalFolder, "localFolder");

        AbbPcSdk sdk = Sdk;

        if (!Directory.Exists(localFolder))
            throw new AbbRemoteHelperException($"ABB program folder was not found: {localFolder}", AbbRemoteErrorCodes.UploadFailed, _log);

        string localProgramFile = Path.Combine(localFolder, $"{programName}_T_ROB1.pgf");

        if (!File.Exists(localProgramFile))
            throw new AbbRemoteHelperException($"ABB remote upload currently supports one RAPID task, T_ROB1. Missing {Path.GetFileName(localProgramFile)}.", AbbRemoteErrorCodes.Unsupported, _log);

        object fileSystem = Get<object>(controller, "FileSystem");
        string robotFolder = Path.Combine(Get<string>(fileSystem, "RemoteDirectory"), programName);
        string robotProgramFile = Path.Combine(robotFolder, $"{programName}_T_ROB1.pgf");

        try
        {
            object authentication = Get<object>(controller, "AuthenticationSystem");
            _ = sdk.Invoke(authentication, "DemandGrant", sdk.StaticValue("ABB.Robotics.Controllers.Grant", "WriteFtp"));
            _ = sdk.Invoke(fileSystem, "PutDirectory", localFolder, programName, true);
            AddLog($"Program {programName} copied to {Get<string>(controller, "Name")}.");
        }
        catch (Exception exception)
        {
            throw new AbbRemoteHelperException($"Could not upload ABB program {programName}: {Unwrap(exception).Message}", AbbRemoteErrorCodes.UploadFailed, _log, exception);
        }

        if (!string.Equals(Get<object>(controller, "OperatingMode").ToString(), "Auto", StringComparison.Ordinal))
            return $"Program {programName} was uploaded. Set controller {Get<string>(controller, "Name")} to Auto mode to load it remotely.";

        LoadProgram(controller, programName, robotProgramFile);
        return $"Program {programName} loaded to {Get<string>(controller, "Name")}.";
    }

    void LoadProgram(object controller, string programName, string robotProgramFile)
    {
        AbbPcSdk sdk = Sdk;

        try
        {
            WithRapidWriteAccess(controller, () =>
            {
                object authentication = Get<object>(controller, "AuthenticationSystem");
                _ = sdk.Invoke(authentication, "DemandGrant", sdk.StaticValue("ABB.Robotics.Controllers.Grant", "LoadRapidProgram"));

                object rapid = Get<object>(controller, "Rapid");
                object task = ((IEnumerable)sdk.Invoke(rapid, "GetTasks")).Cast<object>().First();

                try
                {
                    _ = sdk.Invoke(task, "DeleteProgram");

                    for (int attempt = 1; attempt <= 10; attempt++)
                    {
                        try
                        {
                            object replace = sdk.StaticValue("ABB.Robotics.Controllers.RapidDomain.RapidLoadMode", "Replace");

                            if ((bool)sdk.Invoke(task, "LoadProgramFromFile", robotProgramFile, replace))
                                return;
                        }
                        catch (Exception exception) when (attempt < 10)
                        {
                            AddLog($"Retrying ABB program load ({attempt}/10): {Unwrap(exception).Message}");
                            Thread.Sleep(300);
                        }
                    }

                    throw new AbbRemoteHelperException($"Could not load ABB program {programName} after 10 attempts.", AbbRemoteErrorCodes.LoadFailed, _log);
                }
                finally
                {
                    (task as IDisposable)?.Dispose();
                }
            });
        }
        catch (AbbRemoteHelperException)
        {
            throw;
        }
        catch (Exception exception) when (IsWriteAccessFailure(Unwrap(exception)))
        {
            throw new AbbRemoteHelperException($"Could not acquire ABB write access: {Unwrap(exception).Message}", AbbRemoteErrorCodes.MastershipFailed, _log, exception);
        }
        catch (Exception exception)
        {
            throw new AbbRemoteHelperException($"Could not load ABB program {programName}: {Unwrap(exception).Message}", AbbRemoteErrorCodes.LoadFailed, _log, exception);
        }
    }

    string Play(object controller)
    {
        if (!string.Equals(Get<object>(controller, "OperatingMode").ToString(), "Auto", StringComparison.Ordinal))
            throw new AbbRemoteHelperException("ABB controller must be in Auto mode before starting or resuming a program.", AbbRemoteErrorCodes.InvalidState, _log);

        if (!string.Equals(Get<object>(controller, "State").ToString(), "MotorsOn", StringComparison.Ordinal))
            throw new AbbRemoteHelperException("ABB controller motors must be on before starting or resuming a program.", AbbRemoteErrorCodes.InvalidState, _log);

        try
        {
            AbbPcSdk sdk = Sdk;

            WithRapidWriteAccess(controller, () =>
            {
                object rapid = Get<object>(controller, "Rapid");
                _ = sdk.Invoke(
                    rapid,
                    "Start",
                    sdk.StaticValue("ABB.Robotics.Controllers.RapidDomain.RegainMode", "Continue"),
                    sdk.StaticValue("ABB.Robotics.Controllers.RapidDomain.ExecutionMode", "Continuous"),
                    sdk.StaticValue("ABB.Robotics.Controllers.RapidDomain.ExecutionCycle", "Once"),
                    sdk.StaticValue("ABB.Robotics.Controllers.RapidDomain.StartCheck", "CallChain"));
            });

            return "ABB program started.";
        }
        catch (Exception exception) when (IsWriteAccessFailure(Unwrap(exception)))
        {
            throw new AbbRemoteHelperException($"Could not acquire ABB write access: {Unwrap(exception).Message}", AbbRemoteErrorCodes.MastershipFailed, _log, exception);
        }
    }

    string Pause(object controller)
    {
        if (!string.Equals(Get<object>(controller, "OperatingMode").ToString(), "Auto", StringComparison.Ordinal))
            throw new AbbRemoteHelperException("ABB controller must be in Auto mode before pausing a program.", AbbRemoteErrorCodes.InvalidState, _log);

        try
        {
            AbbPcSdk sdk = Sdk;

            WithRapidWriteAccess(controller, () =>
            {
                object rapid = Get<object>(controller, "Rapid");
                _ = sdk.Invoke(rapid, "Stop", sdk.StaticValue("ABB.Robotics.Controllers.RapidDomain.StopMode", "Instruction"));
            });

            return "ABB program stopped.";
        }
        catch (Exception exception) when (IsWriteAccessFailure(Unwrap(exception)))
        {
            throw new AbbRemoteHelperException($"Could not acquire ABB write access: {Unwrap(exception).Message}", AbbRemoteErrorCodes.MastershipFailed, _log, exception);
        }
    }

    void WithRapidWriteAccess(object controller, Action action)
    {
        AbbPcSdk sdk = Sdk;

        if (Get<Version>(controller, "RobotWareVersion") >= RobotWare8)
        {
            object controlStation = Get<object>(controller, "ControlStation");
            object writeAccessStatus = Get<object>(controlStation, "WriteAccessStatus");

            if (!Get<bool>(writeAccessStatus, "ExternalControlEnabled"))
                throw new AbbRemoteHelperException("ABB external control is not enabled. Enable external control on the FlexPendant before using remote commands.", AbbRemoteErrorCodes.InvalidState, _log);

            if (!Get<bool>(controlStation, "MotionControlEnabled"))
            {
                AddLog("Enabling ABB motion control for this control station.");
                Set(controlStation, "MotionControlEnabled", true);
            }

            _ = sdk.Invoke(controlStation, "RequestWriteAccess");

            try
            {
                action();
            }
            finally
            {
                _ = sdk.Invoke(controlStation, "ReleaseWriteAccess");
            }

            return;
        }

        using IDisposable mastership = (IDisposable)sdk.InvokeStatic("ABB.Robotics.Controllers.Mastership", "Request", controller);
        action();
    }

    static IEnumerable<object> Controllers(object scanner)
    {
        object controllers = Get<object>(scanner, "Controllers");
        return ((IEnumerable)controllers).Cast<object>();
    }

    static T Get<T>(object instance, string name)
    {
        PropertyInfo property = instance.GetType().GetProperty(name, BindingFlags.Instance | BindingFlags.Public)
            ?? throw new MissingMemberException(instance.GetType().FullName, name);

        return (T)property.GetValue(instance)!;
    }

    static void Set(object instance, string name, object? value)
    {
        PropertyInfo property = instance.GetType().GetProperty(name, BindingFlags.Instance | BindingFlags.Public)
            ?? throw new MissingMemberException(instance.GetType().FullName, name);

        property.SetValue(instance, value);
    }

    static bool IsWriteAccessFailure(Exception exception)
    {
        string name = exception.GetType().Name;
        return name.Contains("Mastership", StringComparison.OrdinalIgnoreCase)
            || name.Contains("WriteAccess", StringComparison.OrdinalIgnoreCase);
    }

    static string ErrorCodeFor(Exception exception)
    {
        if (exception is UnauthorizedAccessException)
            return AbbRemoteErrorCodes.PermissionDenied;

        if (exception is InvalidOperationException)
            return AbbRemoteErrorCodes.InvalidState;

        if (IsWriteAccessFailure(exception))
            return AbbRemoteErrorCodes.MastershipFailed;

        return AbbRemoteErrorCodes.Unsupported;
    }

    static Exception Unwrap(Exception exception) =>
        exception is TargetInvocationException { InnerException: not null }
            ? exception.InnerException
            : exception;

    static string Require(string? value, string name) =>
        string.IsNullOrWhiteSpace(value)
            ? throw new AbbRemoteHelperException($"ABB remote request is missing {name}.", AbbRemoteErrorCodes.Unsupported)
            : value;

    void AddLog(string message)
    {
        _log.Add(message);
    }
}

internal sealed class AbbPcSdk
{
    const string PathVariable = "ROBOTS_ABB_PCSDK_DIR";
    const string ControllersAssemblyName = "ABB.Robotics.Controllers.PC";
    static readonly string DefaultDirectory = Path.Combine(
        Environment.GetFolderPath(Environment.SpecialFolder.ProgramFilesX86),
        "ABB",
        "SDK",
        "PCSDK 2026",
        "net10.0");

    readonly Assembly _controllersAssembly;

    AbbPcSdk(Assembly controllersAssembly, string directory)
    {
        _controllersAssembly = controllersAssembly;
        Directory = directory;
    }

    public string Directory { get; }

    public static AbbPcSdk Load()
    {
        string? directory = Environment.GetEnvironmentVariable(PathVariable);

        if (string.IsNullOrWhiteSpace(directory))
            directory = DefaultDirectory;

        directory = Path.GetFullPath(directory);

        string path = Path.Combine(directory, $"{ControllersAssemblyName}.dll");

        if (!File.Exists(path))
            throw new AbbPcSdkLoadException($"ABB remote control requires ABB PC SDK for .NET 10. Install ABB PC SDK 2026 or set {PathVariable} to the folder containing {ControllersAssemblyName}.dll. Expected: {path}");

        AssemblyLoadContext.Default.Resolving += (_, assemblyName) =>
        {
            string dependencyPath = Path.Combine(directory, $"{assemblyName.Name}.dll");
            return File.Exists(dependencyPath)
                ? AssemblyLoadContext.Default.LoadFromAssemblyPath(dependencyPath)
                : null;
        };

        try
        {
            Assembly assembly = AssemblyLoadContext.Default.LoadFromAssemblyPath(path);
            return new(assembly, directory);
        }
        catch (Exception exception)
        {
            throw new AbbPcSdkLoadException($"Could not load ABB PC SDK from {path}: {exception.Message}", exception);
        }
    }

    public object Create(string typeName) =>
        Activator.CreateInstance(Type(typeName))
            ?? throw new MissingMethodException(typeName, ".ctor");

    public object Invoke(object instance, string methodName, params object?[] arguments)
    {
        _ = _controllersAssembly;
        return InvokeCore(instance.GetType(), instance, methodName, arguments);
    }

    public object InvokeStatic(string typeName, string methodName, params object?[] arguments) =>
        InvokeCore(Type(typeName), null, methodName, arguments);

    public object StaticValue(string typeName, string memberName)
    {
        Type type = Type(typeName);

        FieldInfo? field = type.GetField(memberName, BindingFlags.Static | BindingFlags.Public);

        if (field is not null)
            return field.GetValue(null)!;

        PropertyInfo? property = type.GetProperty(memberName, BindingFlags.Static | BindingFlags.Public);

        if (property is not null)
            return property.GetValue(null)!;

        if (type.IsEnum)
            return Enum.Parse(type, memberName);

        throw new MissingMemberException(typeName, memberName);
    }

    Type Type(string typeName) =>
        _controllersAssembly.GetType(typeName)
        ?? throw new TypeLoadException($"ABB PC SDK type was not found: {typeName}");

    static object InvokeCore(Type type, object? instance, string methodName, object?[] arguments)
    {
        MethodInfo[] methods =
        [
            .. type.GetMethods(BindingFlags.Instance | BindingFlags.Static | BindingFlags.Public)
            .Where(method => method.Name == methodName)
            .Where(method => method.GetParameters().Length == arguments.Length)
            .Where(method => ParametersCanAccept(method.GetParameters(), arguments))
        ];

        if (methods.Length == 0)
            throw new MissingMethodException(type.FullName, methodName);

        if (methods.Length > 1)
            throw new AmbiguousMatchException($"ABB PC SDK method call was ambiguous: {type.FullName}.{methodName}");

        MethodInfo method = methods[0];

        return method.Invoke(instance, arguments)!;
    }

    static bool ParametersCanAccept(ParameterInfo[] parameters, object?[] arguments)
    {
        for (int i = 0; i < parameters.Length; i++)
        {
            if (arguments[i] is null)
                continue;

            if (!parameters[i].ParameterType.IsInstanceOfType(arguments[i]))
                return false;
        }

        return true;
    }

}

internal sealed class AbbPcSdkLoadException(string message, Exception? innerException = null)
    : InvalidOperationException(message, innerException)
{ }
