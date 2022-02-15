using System.Runtime.CompilerServices;
using System.ComponentModel;
using System.Windows.Media;
using SharpDX;
using HelixToolkit.Wpf.SharpDX;
using HelixToolkit.Wpf.SharpDX.Controls;

namespace Robots.Samples.Wpf;

class MainViewModel : INotifyPropertyChanged
{
    string _log = "";
    double _time;
    bool _isPlaying;
    Program? _program;
    readonly CompositionTargetEx _timer = new();

    public ObservableElement3DCollection RobotModels { get; } = new();
    public event PropertyChangedEventHandler? PropertyChanged;

    public string Log
    {
        get => _log;
        set => SetField(ref _log, value);
    }

    public double Time
    {
        get => _time;
        set
        {
            if (SetField(ref _time, value))
                _program?.Animate(_time / 100.0);
        }
    }

    public bool IsPlaying
    {
        get => _isPlaying;
        set
        {
            if (!SetField(ref _isPlaying, value))
                return;

            if (_isPlaying)
            {
                _timer.Rendering += Update;
            }
            else
            {
                _timer.Rendering -= Update;
            }
        }
    }

    public async Task InitAsync()
    {
        // robot material
        var material = new PBRMaterial
        {
            AlbedoColor = new Color4(0.8f, 0.4f, 0.05f, 1f),
            MetallicFactor = 0.2,
            ReflectanceFactor = 0.8,
            RenderEnvironmentMap = false,
            AmbientOcclusionFactor = 1
        };

        // robot program
        var programTask = TestProgram.CreateAsync();
        var cancel = new CancellationTokenSource();
        _ = SpinnerAsync(cancel);
        _program = await programTask;
        cancel.Cancel();

        _program.MeshPoser = new HelixMeshPoser(_program.RobotSystem, material, RobotModels);

        Log = _program.Errors.Count == 0
            ? _program.RobotSystem.Name
            : string.Join(" ", _program.Errors);

        IsPlaying = true;
    }

    async Task SpinnerAsync(CancellationTokenSource cancel)
    {
        using var timer = new PeriodicTimer(TimeSpan.FromMilliseconds(250));
        string text = "Downloading Bartlett library";

        while (await timer.WaitForNextTickAsync(cancel.Token))
            Log = text += ".";
    }

    void Update(object? sender, RenderingEventArgs args)
    {
        if (_program is null)
            return;

        var t = args.RenderingTime.TotalSeconds / _program.Duration;
        Time = PingPong(t) * 100;
    }

    static double PingPong(double t)
    {
        t = Math.Clamp(t - Math.Floor(t / 2) * 2, 0.0, 2);
        return 1 - Math.Abs(t - 1);
    }

    bool SetField<T>(ref T field, T value, [CallerMemberName] string property = "")
        where T : notnull, IEquatable<T>
    {
        if (value.Equals(field))
            return false;

        field = value;
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(property));
        return true;
    }
}