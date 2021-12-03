using System.ComponentModel;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Eto.Drawing;
using Eto.Forms;

namespace Robots.Grasshopper;

public sealed class Simulation : GH_Component, IDisposable
{
    readonly AnimForm _form;
    double _speed = 1;
    DateTime _lastTime;
    double _time = 0;
    double _sliderTime = 0;

    public Simulation() : base("Program simulation", "Sim", "Rough simulation of the robot program, right click for playback controls", "Robots", "Components")
    {
        _form = new AnimForm(this)
        {
            Owner = Rhino.UI.RhinoEtoApp.MainWindow
        };
    }

    public override GH_Exposure Exposure => GH_Exposure.quarternary;
    public override Guid ComponentGuid => new("{6CE35140-A625-4686-B8B3-B734D9A36CFC}");
    protected override System.Drawing.Bitmap Icon => Properties.Resources.iconSimulation;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program to simulate", GH_ParamAccess.item);
        pManager.AddNumberParameter("Time", "T", "Advance the simulation to this time", GH_ParamAccess.item, 0);
        pManager.AddBooleanParameter("Normalized", "N", "Time value is normalized (from 0 to 1)", GH_ParamAccess.item, true);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddMeshParameter("System meshes", "M", "System meshes", GH_ParamAccess.list);
        pManager.AddNumberParameter("Joint rotations", "J", "Joint rotations", GH_ParamAccess.list);
        pManager.AddPlaneParameter("Plane", "P", "TCP position", GH_ParamAccess.list);
        pManager.AddIntegerParameter("Index", "I", "Current target index", GH_ParamAccess.item);
        pManager.AddNumberParameter("Time", "T", "Current time in seconds", GH_ParamAccess.item);
        pManager.AddParameter(new ProgramParameter(), "Program", "P", "This is the same program as the input program. Use this output to update other visualization components along with the simulation.", GH_ParamAccess.item);
        pManager.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        GH_Program? program = null;
        GH_Number? sliderTimeGH = null;
        GH_Boolean? isNormalized = null;
        if (!DA.GetData(0, ref program) || program is null) { return; }
        if (!DA.GetData(1, ref sliderTimeGH) || sliderTimeGH is null) { return; }
        if (!DA.GetData(2, ref isNormalized) || isNormalized is null) { return; }

        if (program?.Value is not Program p)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Input program can't have custom code.");
            return;
        }

        _sliderTime = (isNormalized.Value) ? sliderTimeGH.Value * p.Duration : sliderTimeGH.Value;
        if (!_form.Visible) _time = _sliderTime;

        p.Animate(_time, false);
        var currentTarget = p.CurrentSimulationTarget;

        var errors = currentTarget.ProgramTargets.SelectMany(x => x.Kinematics.Errors);
        var joints = currentTarget.ProgramTargets.SelectMany(x => x.Kinematics.Joints);
        var planes = currentTarget.ProgramTargets.SelectMany(x => x.Kinematics.Planes).ToList();
        var meshes = GeometryUtil.PoseMeshes(p.RobotSystem, currentTarget.ProgramTargets.Select(p => p.Kinematics).ToList(), currentTarget.ProgramTargets.Select(p => p.Target.Tool.Mesh).ToList());

        if (errors.Count() > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");

        DA.SetDataList(0, meshes);
        DA.SetDataList(1, joints);
        DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
        DA.SetData(3, currentTarget.Index);
        DA.SetData(4, p.CurrentSimulationTime);
        DA.SetData(5, new GH_Program(p));
        DA.SetDataList(6, errors);

        var isChecked = _form.Play.Checked.HasValue && _form.Play.Checked.Value;

        if (_form.Visible && isChecked)
        {
            var currentTime = DateTime.Now;
            TimeSpan delta = currentTime - _lastTime;
            _time += delta.TotalSeconds * _speed;
            _lastTime = currentTime;
            ExpireSolution(true);
        }
    }

    // Form
    protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
    {
        Menu_AppendItem(menu, "Open controls", OpenForm, true, _form.Visible);
    }

    void OpenForm(object sender, EventArgs e)
    {
        if (_form.Visible)
        {
            _form.Play.Checked = false;
            _form.Visible = false;
        }
        else
        {
            var mousePos = Mouse.Position;
            int x = (int)mousePos.X + 20;
            int y = (int)mousePos.Y - 160;

            _form.Location = new Eto.Drawing.Point(x, y);
            _form.Show();
        }
    }

    void ClickPlay(object sender, EventArgs e)
    {
        _lastTime = DateTime.Now;
        ExpireSolution(true);
    }

    void ClickStop(object sender, EventArgs e)
    {
        _form.Play.Checked = false;
        _time = _sliderTime;
        ExpireSolution(true);
    }

    void ClickScroll(object sender, EventArgs e)
    {
        _speed = (double)_form.Slider.Value / 100.0;
    }

    public void Dispose()
    {
        _form.Dispose();
    }

    class AnimForm : Form
    {
        readonly Simulation _component;

        internal CheckBox Play;
        internal Slider Slider;

        public AnimForm(Simulation component)
        {
            _component = component;

            Maximizable = false;
            Minimizable = false;
            Padding = new Padding(5);
            Resizable = false;
            ShowInTaskbar = true;
            Topmost = true;
            Title = "Playback";
            WindowStyle = WindowStyle.Default;

            var font = new Font(FontFamilies.Sans, 12, FontStyle.None, FontDecoration.None);
            var size = new Size(35, 35);

            Play = new CheckBox()
            {
                Text = "\u25B6",
                Size = size,
                Font = font,
                Checked = false,
                TabIndex = 0
            };
            Play.CheckedChanged += component.ClickPlay;

            var stop = new Button()
            {
                Text = "\u25FC",
                Size = size,
                Font = font,
                TabIndex = 1
            };
            stop.Click += component.ClickStop;

            Slider = new Slider()
            {
                Orientation = Orientation.Vertical,
                Size = new Size(45, 200),
                TabIndex = 2,
                MaxValue = 400,
                MinValue = -200,
                TickFrequency = 100,
                SnapToTick = true,
                Value = 100,
            };
            Slider.ValueChanged += _component.ClickScroll;

            var speedLabel = new Label()
            {
                Text = "100%",
                VerticalAlignment = VerticalAlignment.Center,
            };

            var layout = new DynamicLayout();
            layout.BeginVertical(new Padding(2), Size.Empty);
            layout.AddSeparateRow(padding: new Padding(10), spacing: new Size(10, 0), controls: new Control[] { Play, stop });
            layout.BeginGroup("Speeds");
            layout.AddSeparateRow(Slider, speedLabel);
            layout.EndGroup();
            layout.EndVertical();

            Content = layout;
        }

        protected override void OnClosing(CancelEventArgs e)
        {
            //base.OnClosing(e);
            e.Cancel = true;
            Play.Checked = false;
            Visible = false;
        }
    }
}
