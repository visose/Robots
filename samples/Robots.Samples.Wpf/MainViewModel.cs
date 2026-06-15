using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows.Media;
using HelixToolkit.Wpf.SharpDX;
using HelixToolkit.Wpf.SharpDX.Controls;
using SharpDX;

namespace Robots.Samples.Wpf;

class MainViewModel : INotifyPropertyChanged, IDisposable
{
    Program? _program;
    readonly CompositionTargetEx _timer = new();

    public ObservableElement3DCollection RobotModels { get; } = [];
    public event PropertyChangedEventHandler? PropertyChanged;

    public string Log
    {
        get;
        set
        {
            if (field == value)
                return;

            field = value;
            OnPropertyChanged();
        }
    } = "";

    public double Time
    {
        get;
        set
        {
            if (field == value)
                return;

            field = value;
            OnPropertyChanged();
            _program?.Animate(value / 100.0);
        }
    }

    public bool IsPlaying
    {
        get;
        set
        {
            if (field == value)
                return;

            field = value;
            OnPropertyChanged();

            if (value)
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

    void OnPropertyChanged([CallerMemberName] string property = "")
    {
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(property));
    }

    public void Dispose()
    {
        _timer.Rendering -= Update;
        _timer.Dispose();
        GC.SuppressFinalize(this);
    }
}
