using System.Xml;
using Rhino.Geometry;
using Grasshopper.Kernel;

namespace Robots.Grasshopper;

public class LoadRobotSystem : GH_Component
{
    LibraryForm? _form;

    public LoadRobotSystem() : base("Load robot system", "LoadRobot", "Loads a robot system from the library.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconRobot");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name of the robot system", GH_ParamAccess.item);
        pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item, Plane.WorldXY);
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
    }

    public override void AddedToDocument(GH_Document document)
    {
        base.AddedToDocument(document);
        LibraryParam.CreateIfEmpty(document, this, ElementType.RobotCell);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;
        Plane basePlane = default;

        if (!DA.GetData(0, ref name)) return;
        if (!DA.GetData(1, ref basePlane)) return;

        if (name is null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, $" Input parameter N cannot be null");
            DA.AbortComponentSolution();
            return;
        }

        try
        {
            var robotSystem = FileIO.LoadRobotSystem(name, basePlane);
            DA.SetData(0, robotSystem);
        }
        catch (Exception e)
        {
            var message = e is XmlException ex
                ? $" Invalid XML format in \"{Path.GetFileName(ex.SourceUri)}\""
                : e.Message;

            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, message);
            DA.AbortComponentSolution();
        }
    }

    // form

    public override void CreateAttributes()
    {
        m_attributes = new ComponentButton(this, "Libraries", ToggleForm);
    }

    public override void RemovedFromDocument(GH_Document document)
    {
        base.RemovedFromDocument(document);

        if (_form is not null)
            _form.Visible = false;

        if (LibraryParam.IsConnected(this, out var libraryParam))
            document.RemoveObject(libraryParam, false);
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
}