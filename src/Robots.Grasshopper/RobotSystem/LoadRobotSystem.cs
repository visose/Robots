using Rhino.Geometry;

namespace Robots.Grasshopper;

public class LoadRobotSystem() : Component(
    "Load Robot System",
    "Loads a robot system from the library.",
    "Components",
    "{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}"), IDisposable
{
    LibraryForm? _form;

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        _ = pManager.AddTextParameter("Name", "N", "Robot system name in the library.", GH_ParamAccess.item);
        _ = pManager.AddPlaneParameter("Base", "P", "Robot system base plane.", GH_ParamAccess.item, Plane.WorldXY);
        _ = pManager.AddParameter(new PostProcessorParameter(), "Post Processor", "Ps", "Optional alternative post processor.", GH_ParamAccess.item);
        pManager[2].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        _ = pManager.AddParameter(new RobotSystemParameter(), "Robot System", "R", "Loaded robot system.", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        _ = LibraryParam.CreateIfEmpty(document, this, ElementType.RobotSystem);
    }

    protected override void SolveComponent(IGH_DataAccess DA)
    {
        var robotSystem = FileIO.LoadRobotSystem(DA.Get<string>(0), DA.Get<Plane>(1), postProcessor: DA.Maybe<IPostProcessor>(2));
        _ = DA.SetData(0, robotSystem);
    }

    public override void CreateAttributes()
    {
        m_attributes = new ComponentButton(this, "Libraries", ToggleForm);
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        base.RemovedFromDocument(document);

        _ = (_form?.Visible = false);

        if (LibraryParam.IsConnected(this, out var libraryParam))
            _ = document.RemoveObject(libraryParam, false);
    }

    void ToggleForm()
    {
        if (_form is null)
        {
            var library = new OnlineLibrary();
            library.LibraryChanged += () =>
            {
                if (LibraryParam.IsConnected(this, out var libraryParam))
                    libraryParam.UpdateAndExpire();
            };

            _form = new LibraryForm(library);
        }

        _form.Visible = !_form.Visible;
    }

    public void Dispose()
    {
        _form?.Dispose();
        _form = null;
        GC.SuppressFinalize(this);
    }
}
