using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public sealed class Simulation : GH_Component
{
    SimulationForm? _form;
    DateTime? _lastTime;
    double _time = 0;
    double _lastInputTime = 0;

    internal double Speed = 1;

    public Simulation() : base("Program simulation", "Sim", "Rough simulation of the robot program, right click for playback controls", "Robots", "Components") { }

    public override GH_Exposure Exposure => GH_Exposure.quinary;
    public override Guid ComponentGuid => new("{6CE35140-A625-4686-B8B3-B734D9A36CFC}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconSimulation");

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
        IProgram? program = null;
        double inputTime = 0;
        bool isNormalized = true;

        if (!DA.GetData(0, ref program) || program is null) return;
        if (!DA.GetData(1, ref inputTime)) return;
        if (!DA.GetData(2, ref isNormalized)) return;

        if (!program.HasSimulation)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, " Input program cannot be animated");
            DA.AbortComponentSolution();
            return;
        }

        var p = (Program)program;
        inputTime = (isNormalized) ? inputTime * p.Duration : inputTime;

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

        p.MeshPoser ??= new RhinoMeshPoser(p.RobotSystem);
        p.Animate(_time, false);

        var currentPose = p.CurrentSimulationPose;
        var currentKinematics = currentPose.Kinematics;
        var currentCellTarget = p.Targets[currentPose.TargetIndex];

        var errors = currentKinematics.SelectMany(x => x.Errors);
        var joints = currentKinematics.SelectMany(x => x.Joints);
        var planes = currentKinematics.SelectMany(x => x.Planes).ToList();
        var meshes = ((RhinoMeshPoser)p.MeshPoser).Meshes;

        if (errors.Any())
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");

        DA.SetDataList(0, meshes);
        DA.SetDataList(1, joints);
        DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
        DA.SetData(3, currentPose.TargetIndex);
        DA.SetData(4, currentPose.CurrentTime);
        DA.SetData(5, program);
        DA.SetDataList(6, errors);

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
        if (_form is not null)
            _form.Play.Checked = false;

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

    // form

    public override void CreateAttributes()
    {
        m_attributes = new ComponentButton(this, "Playback", ToggleForm);
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        base.RemovedFromDocument(document);

        if (_form is not null)
            _form.Visible = false;
    }

    void ToggleForm()
    {
        _form ??= new SimulationForm(this);
        _form.Visible = !_form.Visible;

        if (!_form.Visible)
            Stop();
    }
}