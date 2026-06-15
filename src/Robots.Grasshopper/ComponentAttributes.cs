using System.Drawing;

using Grasshopper;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel.Attributes;

namespace Robots.Grasshopper;

class ComponentAttributes(IGH_Component owner) : GH_ComponentAttributes(owner)
{
    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        if (channel != GH_CanvasChannel.Objects || !IsIconMode(Owner.IconDisplayMode))
        {
            base.Render(canvas, graphics, channel);
            return;
        }

        RenderComponentCapsule(
            canvas,
            graphics,
            drawComponentBaseBox: true,
            drawComponentNameBox: false,
            drawJaggedEdges: true,
            drawParameterGrips: true,
            drawParameterNames: true,
            drawZuiElements: true);

        RenderIcon(canvas, graphics);
    }

    void RenderIcon(GH_Canvas canvas, Graphics graphics)
    {
        var icon = Owner.Locked
            ? Owner.Icon_24x24_Locked ?? Owner.Icon_24x24
            : Owner.Icon_24x24;

        if (icon is null || GH_Canvas.ZoomFadeLow < 5)
            return;

        var box = ContentBox;
        IconDrawing.RenderScaledIcon(graphics, icon, box);

        if (Owner.Obsolete && CentralSettings.CanvasObsoleteTags && canvas.DrawingMode == GH_CanvasMode.Control)
            GH_GraphicsUtil.RenderObjectOverlay(graphics, Owner, box);
    }
}
