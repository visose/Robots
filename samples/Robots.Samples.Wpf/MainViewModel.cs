using System.ComponentModel;
using System.Runtime.CompilerServices;
using SharpDX;
using HelixToolkit.Wpf.SharpDX;

namespace Robots.Samples.Wpf;

public class MainViewModel : INotifyPropertyChanged
{
    string _log = "";
    double _time;
    bool _isPlaying;
    DateTime _last;
    double _dir = 1.0;

    Program? _program;
    readonly Timer _timer;

    public event PropertyChangedEventHandler? PropertyChanged;
    public ObservableElement3DCollection RobotModels { get; } = new();

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
            SetField(ref _time, value);
            _program?.Animate(_time / 100.0);
        }
    }

    public bool IsPlaying
    {
        get => _isPlaying;
        set
        {
            SetField(ref _isPlaying, value);

            if (_isPlaying)
            {
                _last = DateTime.Now;
                _timer.Change(0, 16);
            }
            else
            {
                _timer.Change(Timeout.Infinite, 0);
            }
        }
    }

    public MainViewModel()
    {
        _timer = new Timer(Tick, null, Timeout.Infinite, 0);
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
        _ = SpinnerAsync();

        _program = await programTask;
        _program.MeshPoser = new HelixMeshPoser(_program.RobotSystem, material, RobotModels);
    }

    async Task SpinnerAsync()
    {
        string dots = "...";

        while(_program is null)
        {
            Log = "Downloading Bartlett library " + dots;
            dots += ".";
            await Task.Delay(250);
        }

        Log = _program.Errors.Count == 0 
            ? _program.RobotSystem.Name 
            : string.Join(" ", _program.Errors);
    }


    void Tick(object? o)
    {
        if (_program is null)
            return;

        if (_time < 0 || _time > 100)
            _dir *= -1;

        var now = DateTime.Now;
        var delta = now - _last;
        var t = (delta.TotalSeconds / _program.Duration) * 100;
        Time += t * _dir;
        _last = now;
    }

    void SetField<T>(ref T field, T value, [CallerMemberName] string property = "")
    {
        field = value;
        PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(property));
    }
}