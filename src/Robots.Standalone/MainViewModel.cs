using SharpDX;
using HelixToolkit.Wpf.SharpDX;

namespace Robots.Standalone;

public class MainViewModel : ObservableObject
{
    string _log = "";
    double _time;
    bool _isPlaying;

    readonly Timer _timer;
    DateTime _last;
    double _dir = 1.0;

    public ObservableElement3DCollection RobotModels { get; } = new ObservableElement3DCollection();
    public Program Program { get; set; }

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
            Program.Animate(_time / 100.0);
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
        Program = TestProgram.Create();
        Log = Program.Errors.Count == 0 ? Program.RobotSystem.Name : string.Join(" ", Program.Errors);
        Program.MeshPoser = new HelixMeshPoser((RobotCell)Program.RobotSystem, material, RobotModels);

        // timer
        _timer = new Timer(Tick, null, Timeout.Infinite, 0);
    }

    void Tick(object? o)
    {
        if (_time < 0 || _time > 100)
            _dir *= -1;

        var now = DateTime.Now;
        var delta = now - _last;
        var t = (delta.TotalSeconds / Program.Duration) * 100;
        Time += t * _dir;
        _last = now;
    }
}