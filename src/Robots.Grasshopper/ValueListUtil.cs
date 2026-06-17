using Grasshopper;
using Grasshopper.Kernel.Special;

namespace Robots.Grasshopper;

static class ValueListUtil
{
    public static void AddToInput(GH_Component component, IGH_Param parameter, GH_ValueList list, int xOffset, int yOffset)
    {
        LayoutComponent(component);

        if (parameter.Attributes is null)
            parameter.CreateAttributes();

        var parameterAttributes = parameter.Attributes
            ?? throw new InvalidOperationException("Could not create input parameter attributes.");

        list.CreateAttributes();
        list.Attributes.Pivot = new(parameterAttributes.InputGrip.X + xOffset, parameterAttributes.InputGrip.Y + yOffset);
        Add(FindDocument(component), parameter, list);
    }

    public static void AddNearComponent(GH_Document document, GH_Component component, IGH_Param parameter, GH_ValueList list, int xOffset, int yOffset)
    {
        LayoutComponent(component);

        var componentAttributes = component.Attributes
            ?? throw new InvalidOperationException("Could not create component attributes.");

        list.CreateAttributes();
        list.Attributes.Pivot = new(componentAttributes.Pivot.X + xOffset, componentAttributes.Pivot.Y + yOffset);
        Add(document, parameter, list);
    }

    static void LayoutComponent(GH_Component component)
    {
        component.Params.OnParametersChanged();

        var attributes = component.Attributes;

        if (attributes is null)
        {
            component.CreateAttributes();
            attributes = component.Attributes
                ?? throw new InvalidOperationException("Could not create component attributes.");
        }

        attributes.PerformLayout();
    }

    static void Add(GH_Document document, IGH_Param parameter, GH_ValueList list)
    {
        _ = document.AddObject(list, false);
        parameter.AddSource(list);
        parameter.CollectData();
    }

    static GH_Document FindDocument(GH_Component component) =>
        component.OnPingDocument()
        ?? Instances.ActiveCanvas?.Document
        ?? throw new InvalidOperationException("Could not find the active Grasshopper document.");
}
