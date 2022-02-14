using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using SharpDX;
using HelixToolkit.SharpDX.Core;
using HelixToolkit.Wpf.SharpDX;
using PerspectiveCamera = HelixToolkit.Wpf.SharpDX.PerspectiveCamera;
using Color = System.Windows.Media.Color;

namespace Robots.Samples.Wpf;

public partial class MainWindow : Window, IDisposable
{
    public MainWindow()
    {
        InitializeComponent();
        SetViewport();
    }

    void SetViewport()
    {
        view.EnableDesignModeRendering = false;
        view.FrameRate = 60;
        view.MSAA = MSAALevel.Four;
        view.FXAALevel = FXAALevel.High;
        view.EnableDeferredRendering = true;
        view.IsShadowMappingEnabled = false;
        view.EnableSwapChainRendering = true;
        view.EnableAutoOctreeUpdate = false;
        view.ShowViewCube = false;
        view.ShowCoordinateSystem = true;
        view.CoordinateSystemLabelForeground = Colors.White;
        view.EffectsManager = new DefaultEffectsManager();
        view.TextBrush = new SolidColorBrush(Colors.White);
        view.BackgroundColor = Color.FromRgb(40, 40, 40);
        view.CameraInertiaFactor = 0.2;
        view.ZoomSensitivity = 4.0;
        view.RotationSensitivity = 0.5;
        view.ModelUpDirection = new Vector3D(0, 0, 1);
        view.Camera = new PerspectiveCamera
        {
            Position = new Point3D(0, -2000, 400),
            LookDirection = new Vector3D(0, 2000, 0),
            UpDirection = new Vector3D(0, 0, 1),
            FarPlaneDistance = 10000,
            NearPlaneDistance = 10
        };
        view.CameraChanged += (o, e) => SetHeadLight();

        ambient.Color = Color.FromScRgb(1, 0.02f, 0.02f, 0.02f);

        light.Color = Colors.White;
        var lightDir = new Vector3D(0, -0.5, -1);
        lightDir.Normalize();
        light.Direction = lightDir;
        SetHeadLight();

        grid.Geometry = LineBuilder.GenerateGrid(new Vector3(0, 0, 1), -10, 10, -10, 10);
        grid.Color = Color.FromRgb(100, 100, 100);
        grid.Transform = new ScaleTransform3D(100, 100, 100);
        grid.Thickness = 0.5;
    }

    void SetHeadLight()
    {
        var matrix = view.Camera.GetInversedViewMatrix();
        var t = new MatrixTransform3D(matrix);
        light.Transform = t;
    }

    private bool _disposedValue = false;
    public void Dispose() => Dispose(true);

    protected virtual void Dispose(bool disposing)
    {
        if (!_disposedValue)
        {
            var effectsManager = view.EffectsManager;

            if (effectsManager is not null)
                Disposer.RemoveAndDispose(ref effectsManager);

            _disposedValue = true;
            GC.SuppressFinalize(this);
        }
    }

    ~MainWindow()
    {
        Dispose(false);
    }

}