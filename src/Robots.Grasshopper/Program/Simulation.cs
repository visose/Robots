
namespace Robots.Grasshopper;

public sealed class Simulation() : Component(
    "Program Simulation",
    "Simulates a checked robot program. Right-click for playback controls.",
    "Components",
    "{6CE35140-A625-4686-B8B3-B734D9A36CFC}",
    GH_Exposure.quinary)
{
    SimulationForm? _form;
    DateTime? _lastTime;
    double _time;
    double _lastInputTime;

    internal double Speed = 1;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program to simulate.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Time", "T", "Simulation time. Use 0 to 1 when Normalized is true.", GH_ParamAccess.item, 0);
        _ = pManager.AddBooleanParameter("Normalized", "N", "Treat Time as a normalized value from 0 to 1.", GH_ParamAccess.item, true);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddMeshParameter("System Meshes", "M", "Robot system meshes at the current time.", GH_ParamAccess.list);
        _ = pManager.AddNumberParameter("Joint Rotations", "J", "Joint rotations in radians at the current time.", GH_ParamAccess.list);
        _ = pManager.AddPlaneParameter("Plane", "P", "TCP planes at the current time.", GH_ParamAccess.list);
        _ = pManager.AddIntegerParameter("Index", "I", "Current target index.", GH_ParamAccess.item);
        _ = pManager.AddNumberParameter("Time", "T", "Current time in seconds.", GH_ParamAccess.item);
        _ = pManager.AddParameter(new ProgramParameter(), "Program", "P", "Pass-through program for visualization components that use simulation state.", GH_ParamAccess.item);
        _ = pManager.AddTextParameter("Errors", "E", "Simulation errors.", GH_ParamAccess.list);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var inputProgram = DA.Get<IProgram>(0);
        var inputTime = DA.Get<double>(1);
        var isNormalized = DA.Get<bool>(2);

        if (inputProgram is not Program p || !p.HasSimulation)
            throw new RuntimeWarningException("Input program cannot be animated.");

        inputTime = isNormalized ? inputTime * p.Duration : inputTime;

        if (_lastInputTime != inputTime)
        {
            if (_lastTime is not null)
                Pause();

            _time = inputTime;
            _lastInputTime = inputTime;
        }

        if (_time < 0 || _time > p.Duration)
        {
            _time = Rhino.RhinoMath.Clamp(_time, 0, p.Duration);
            Pause();
        }

        if (p.MeshPoser is not RhinoMeshPoser meshPoser)
        {
            meshPoser = new(p.RobotSystem);
            p.MeshPoser = meshPoser;
        }

        p.Animate(_time, false);

        var currentPose = p.CurrentSimulationPose;
        var currentKinematics = currentPose.Kinematics;
        var errors = currentKinematics.AllErrors();

        if (errors.Length > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Solution has errors.");

        _ = DA.SetDataList(0, meshPoser.Meshes);
        _ = DA.SetDataList(1, currentKinematics.AllJoints());
        _ = DA.SetDataList(2, currentKinematics.AllPlanes());
        _ = DA.SetData(3, currentPose.TargetIndex);
        _ = DA.SetData(4, currentPose.CurrentTime);
        _ = DA.SetData(5, inputProgram);
        _ = DA.SetDataList(6, errors);

        Update();
    }

    internal void TogglePlay()
    {
        if (_lastTime is null)
        {
            _lastTime = DateTime.Now;
            ExpireSolution(true);
        }
        else
        {
            Pause();
        }
    }

    internal void Stop()
    {
        Pause();
        _time = _lastInputTime;
        ExpireSolution(true);
    }

    void Pause()
    {
        _ = (_form?.Play.Checked = false);

        _lastTime = null;
    }

    void Update()
    {
        if (_lastTime is null)
            return;

        var currentTime = DateTime.Now;
        TimeSpan delta = currentTime - _lastTime.Value;
        _lastTime = currentTime;
        _time += delta.TotalSeconds * Speed;
        ExpireSolution(true);
    }

    public override void CreateAttributes()
    {
        m_attributes = new ComponentButton(this, "Playback", ToggleForm);
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        base.RemovedFromDocument(document);

        _ = (_form?.Visible = false);
    }

    void ToggleForm()
    {
        _form ??= new(this);
        _form.Visible = !_form.Visible;

        if (!_form.Visible)
            Stop();
    }
}
