using SharpDX;
using HelixToolkit.Wpf.SharpDX;

namespace Robots.Samples.Wpf;

public class MainViewModel : ObservableObject
{
    string _log = "";
    double _time;
    bool _isPlaying;
    DateTime _last;
    double _dir = 1.0;

    readonly Program _program;
    readonly Timer _timer;

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
            _program.Animate(_time / 100.0);
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
        _program = TestProgram.Create();
        _program.MeshPoser = new HelixMeshPoser(_program.RobotSystem, material, RobotModels);

        Log = _program.Errors.Count == 0 ? _program.RobotSystem.Name : string.Join(" ", _program.Errors);

        // timer
        _timer = new Timer(Tick, null, Timeout.Infinite, 0);
    }

    void Tick(object? o)
    {
        if (_time < 0 || _time > 100)
            _dir *= -1;

        var now = DateTime.Now;
        var delta = now - _last;
        var t = (delta.TotalSeconds / _program.Duration) * 100;
        Time += t * _dir;
        _last = now;
    }
}