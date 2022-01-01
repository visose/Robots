using System.Drawing;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Special;

namespace Robots.Grasshopper;

public class LoadTool : GH_Component
{
    GH_ValueList? _valueList = null;
    IGH_Param? _parameter = null;

    public LoadTool() : base("Load tool", "LoadTool", "Loads a tool from the library.", "Robots", "Components") { }
    public override GH_Exposure Exposure => GH_Exposure.primary;
    public override Guid ComponentGuid => new("{542aa5fd-4f02-4ee5-a2a0-02b0fac8777f}");
    protected override Bitmap Icon => Util.GetIcon("iconTool");

    protected override void RegisterInputParams(GH_InputParamManager pManager)
    {
        pManager.AddTextParameter("Name", "N", "Name of the tool", GH_ParamAccess.item);
        _parameter = pManager[0];
    }

    protected override void RegisterOutputParams(GH_OutputParamManager pManager)
    {
        pManager.AddParameter(new ToolParameter(), "Tool", "T", "Tool", GH_ParamAccess.item);
    }

    protected override void BeforeSolveInstance()
    {
        if (_parameter is null)
            throw new ArgumentNullException(nameof(_parameter));

        if (_valueList is not null)
            return;

        var inputValueList = _parameter.Sources.FirstOrDefault(s => s is GH_ValueList) as GH_ValueList;
        _valueList = inputValueList ?? new GH_ValueList();

        if (inputValueList is null)
        {
            _valueList.CreateAttributes();
            _valueList.Attributes.Pivot = new PointF(Attributes.Pivot.X - 180, Attributes.Pivot.Y - 31);
            AddToolsToValueList(_valueList);
            Instances.ActiveCanvas.Document.AddObject(_valueList, false);
            _parameter.AddSource(_valueList);
            _parameter.CollectData();
        }
        else
        {
            AddToolsToValueList(_valueList);
        }
    }

    void AddToolsToValueList(GH_ValueList valueList)
    {
        var selected = valueList.FirstSelectedItem;
        var tools = FileIO.ListTools();

        if (tools.SequenceEqual(valueList.ListItems.Select(i => i.Name)))
            return;

        valueList.ListItems.Clear();

        foreach (string toolName in tools)
            valueList.ListItems.Add(new GH_ValueListItem(toolName, $"\"{toolName}\""));

        if (selected is null)
            return;

        var selectedIndex = valueList.ListItems.FindIndex(s => s.Name == selected.Name);

        if (selectedIndex != -1)
            valueList.SelectItem(selectedIndex);
    }

    protected override void SolveInstance(IGH_DataAccess DA)
    {
        string? name = null;

        if (!DA.GetData(0, ref name) || name is null) { return; }

        var tool = FileIO.LoadTool(name);
        DA.SetData(0, new GH_Tool(tool));
    }
}
