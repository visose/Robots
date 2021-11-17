using System;
using System.Linq;
using HelixToolkit.Wpf.SharpDX;
using System.IO;
using SharpDX;
using Vector3D = System.Windows.Media.Media3D.Vector3D;
using Point3D = System.Windows.Media.Media3D.Point3D;
using Color = System.Windows.Media.Color;
using Colors = System.Windows.Media.Colors;
using Transform3D = System.Windows.Media.Media3D.Transform3D;
using System.Windows.Input;
using Rhino.Geometry;
using Plane = Rhino.Geometry.Plane;
using System.Threading;

namespace RobotsStandalone
{
    public class MainViewModel : ObservableObject, IDisposable
    {
        public string _log;

        public string Log
        {
            get { return _log; }
            set { _log = value; OnPropertyChanged(); }
        }

        public double _time;

        public double Time
        {
            get { return _time; }
            set
            {
                _time = value;
                Animate(_time);
                OnPropertyChanged();
            }
        }

        public bool _isPlaying;

        public bool IsPlaying
        {
            get { return _isPlaying; }
            set
            {
                _isPlaying = value;

                if (_isPlaying)
                {
                    _last = DateTime.Now;
                    _timer.Change(0, 1);
                }
                else
                {
                    _timer.Change(Timeout.Infinite, 0);
                }


                OnPropertyChanged();
            }
        }

        Transform3D _lightTransform;
        public Transform3D LightTransform
        {
            get { return _lightTransform; }
            set
            {
                _lightTransform = value;
                OnPropertyChanged();
            }
        }

        public Vector3D LightDirection { get; set; }

        public ObservableElement3DCollection RobotModels { get; } = new ObservableElement3DCollection();
        public Robots.Program Program { get; set; }

        public IEffectsManager EffectsManager { get; }
        public Camera Camera { get; set; }
        public Vector3D UpDirection { get; } = new Vector3D(0, 0, 1);

        public Transform3D ModelTransform { get; }
        public Transform3D EnvironmentTransform { get; }
        public Color AmbientLightColor { get; }
        public Color DirectionalLightColor { get; }
        public Color BackgroundColor { get; }
        public Stream SkyboxTexture { get; }

        public LineGeometry3D Grid { get; }
        public Color GridColor { get; }
        public Transform3D GridTransform { get; }

        public ICommand UpZCommand { get; }

        Viewport3DX _viewport;
        public Viewport3DX Viewport
        {
            get { return _viewport; }
            set
            {
                _viewport = value;
                _viewport.CameraInertiaFactor = 0.2;
                _viewport.ZoomSensitivity = 4.0;
                _viewport.RotationSensitivity = 0.5;

                _viewport.CameraChanged += (o, e) =>
                {
                    SetHeadLight();
                };
            }
        }

        void SetHeadLight()
        {
            var matrix = Camera.GetInversedViewMatrix();
            var t = new System.Windows.Media.Media3D.MatrixTransform3D(matrix);
            LightTransform = t;
        }

        private Timer _timer;

        public MainViewModel()
        {
            EffectsManager = new DefaultEffectsManager();
            // camera setup
            Camera = new PerspectiveCamera
            {
                Position = new Point3D(0, -2000, 400),
                LookDirection = new Vector3D(0, 2000, 0),
                UpDirection = UpDirection,
                FarPlaneDistance = 5000000,
            };


            // setup lighting   
            //SkyboxTexture = LoadFileToMemory("Cubemap_Grandcanyon.dds");
            LightDirection = new Vector3D(0, -0.5, -1);
            LightDirection.Normalize();
            SetHeadLight();

            AmbientLightColor = Colors.White*1000;
            DirectionalLightColor = Color.FromScRgb(1, 10, 10, 10);
            BackgroundColor = Color.FromRgb(40, 40, 40);

            // model transform
            ModelTransform = Transform3D.Identity;
            //var r = new System.Windows.Media.Media3D.AxisAngleRotation3D(new Vector3D(0, 0, 1), 90);
            //EnvironmentTransform = new System.Windows.Media.Media3D.RotateTransform3D(r);

            // model materials
            var material = new PBRMaterial();
            material.AlbedoColor = new Color4(0.8f, 0.4f, 0.05f, 1f);
            material.MetallicFactor = 0.2;
            material.ReflectanceFactor = 0.8;
            material.RenderEnvironmentMap = false;
            material.AmbientOcclusionFactor = 1;

            // floor plane grid
            Grid = LineBuilder.GenerateGrid(new Vector3(0, 0, 1), -10, 10, -10, 10);
            GridColor = Color.FromRgb(100, 100, 100);
            GridTransform = new System.Windows.Media.Media3D.ScaleTransform3D(100, 100, 100);

            // commands
            UpZCommand = new RelayCommand(_ => { Log = "test switch"; });

            // robot program
            var test = new TestProgram();
            Program = test.Program;
            Log = Program.Errors.Count == 0 ? Program.RobotSystem.Name : string.Join(" ", Program.Errors);

            // make robot
            var cell = Program.RobotSystem as Robots.RobotCell;
            var group = cell.MechanicalGroups.First();
            var meshes = group.Joints.Select(j => j.Mesh).Prepend(group.Robot.BaseMesh);
            _origins = group.Joints.Select(j => j.Plane).Prepend(group.Robot.BasePlane).ToArray();

            foreach (var joint in meshes)
            {
                var model = new MeshGeometryModel3D();
                model.Geometry = joint.ToWPF();
                model.Material = material;
                model.Transform = Transform3D.Identity;
                model.IsThrowingShadow = true;
                RobotModels.Add(model);
            }

            // timer
            _timer = new Timer(Tick, null, Timeout.Infinite, 0);
        }

        Plane[] _origins;

        void Animate(double time)
        {
            Program.Animate(time / 100.0);
            var planes = Program.CurrentSimulationTarget.ProgramTargets[0].Kinematics.Planes;

            for (int i = 0; i < planes.Length - 1; i++)
            {
                var s = Transform.PlaneToPlane(_origins[i], planes[i]);
                var matrix = new SharpDX.Matrix((float)s.M00, (float)s.M10, (float)s.M20, (float)s.M30, (float)s.M01, (float)s.M11, (float)s.M21, (float)s.M31, (float)s.M02, (float)s.M12, (float)s.M22, (float)s.M32, (float)s.M03, (float)s.M13, (float)s.M23, (float)s.M33);
                var model = RobotModels[i];
                model.SceneNode.ModelMatrix = matrix;
            }
        }

        DateTime _last;
        double _dir = 1.0;

        void Tick(object o)
        {
            if (_time < 0 || _time > 100)
                _dir *= -1;

            var now = DateTime.Now;
            var delta = now - _last;
            var t = (delta.TotalSeconds / Program.Duration) * 100;
            Time += t * _dir;
            _last = now;
        }

        public static MemoryStream LoadFileToMemory(string filePath)
        {
            using (var file = new FileStream(filePath, FileMode.Open))
            {
                var memory = new MemoryStream();
                file.CopyTo(memory);
                return memory;
            }
        }

        #region IDisposable Support
        private bool disposedValue = false; // To detect redundant calls

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    // TODO: dispose managed state (managed objects).
                }

                // TODO: free unmanaged resources (unmanaged objects) and override a finalizer below.
                // TODO: set large fields to null.
                if (EffectsManager != null)
                {
                    var effectManager = EffectsManager as IDisposable;
                    Disposer.RemoveAndDispose(ref effectManager);
                }
                disposedValue = true;
                GC.SuppressFinalize(this);
            }
        }

        // TODO: override a finalizer only if Dispose(bool disposing) above has code to free unmanaged resources.
        ~MainViewModel()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(false);
        }

        // This code added to correctly implement the disposable pattern.
        public void Dispose()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(true);
            // TODO: uncomment the following line if the finalizer is overridden above.
            // GC.SuppressFinalize(this);
        }
        #endregion
    }
}