﻿using Rhino.Geometry;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.Kernel.Special;

namespace Robots.Grasshopper;

public class LoadRobotSystem : GH_Component
{
    LibraryForm? _form;
    GH_ValueList? _valueList = null;
    IGH_Param? _parameter = null;

    public LoadRobotSystem() : base("Load robot system", "LoadRobot", "Loads a robot system from the library.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{7722D7E3-98DE-49B5-9B1D-E0D1B938B4A7}");
    protected override System.Drawing.Bitmap Icon => Util.GetIcon("iconRobot");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name of the robot system", GH_ParamAccess.item);
        pManager.AddPlaneParameter("Base", "P", "Base plane", GH_ParamAccess.item, Plane.WorldXY);
        _parameter = pManager[0];
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
    }

    protected override void BeforeSolveInstance()
    {
        if (_parameter is null)
            throw new ArgumentNullException(nameof(_parameter));

        if (_valueList is not null)
            return;

        if (_parameter.Sources.FirstOrDefault(s => s is GH_ValueList) is not GH_ValueList inputValueList)
        {
            _valueList = new GH_ValueList();
            _valueList.CreateAttributes();
            _valueList.Attributes.Pivot = new System.Drawing.PointF(Attributes.Pivot.X - 240, Attributes.Pivot.Y - 21);
            UpdateValueList();
            Instances.ActiveCanvas.Document.AddObject(_valueList, false);
            _parameter.AddSource(_valueList);
            _parameter.CollectData();
        }
        else
        {
            _valueList = inputValueList;
            UpdateValueList();
        }
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;
        GH_Plane? basePlane = null;

        if (!DA.GetData(0, ref name) || name is null) { return; }
        if (!DA.GetData(1, ref basePlane) || basePlane is null) { return; }

        try
        {
            var robotSystem = FileIO.LoadRobotSystem(name, basePlane.Value);
            DA.SetData(0, new GH_RobotSystem(robotSystem));
        }
        catch (Exception e)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, e.Message);
        }
    }

    void UpdateValueList()
    {
        if (_valueList is null)
            return;

        var valueList = _valueList;

        var selected = valueList.FirstSelectedItem;
        var robotSystems = FileIO.ListRobotSystems();

        if (robotSystems.SequenceEqual(valueList.ListItems.Select(i => i.Name)))
            return;

        valueList.ListItems.Clear();

        foreach (string robotSystemName in robotSystems)
            valueList.ListItems.Add(new GH_ValueListItem(robotSystemName, $"\"{robotSystemName}\""));

        if (selected is null)
            return;

        var selectedIndex = valueList.ListItems.FindIndex(s => s.Name == selected.Name);

        if (selectedIndex != -1)
            valueList.SelectItem(selectedIndex);
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
    }

    void ToggleForm()
    {
        if (_form is null)
        {
            var library = new OnlineLibrary();
            library.LibraryChanged += UpdateValueList;
            _form = new LibraryForm(library);
        }

        _form.Visible = !_form.Visible;
    }
}