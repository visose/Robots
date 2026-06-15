using System.Drawing;

using Grasshopper;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel.Attributes;

namespace Robots.Grasshopper;

class ParameterAttributes(IGH_Param owner) : GH_FloatingParamAttributes(owner)
{
    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        if (channel != GH_CanvasChannel.Objects || !IsIconMode(Owner.IconDisplayMode))
        {
            base.Render(canvas, graphics, channel);
            return;
        }

        base.Render(canvas, graphics, channel);
        RenderIcon(canvas, graphics);
    }

    void RenderIcon(GH_Canvas _, Graphics graphics)
    {
        var icon = Owner.Locked
            ? Owner.Icon_24x24_Locked ?? Owner.Icon_24x24
            : Owner.Icon_24x24;

        if (icon is null || GH_Canvas.ZoomFadeLow < 5)
            return;

        var iconBounds = Bounds;
        var tagBounds = Owner.StateTags.BoundingBox;

        if (!tagBounds.IsEmpty && tagBounds.IntersectsWith(GH_Convert.ToRectangle(Bounds)))
            iconBounds = RectangleF.FromLTRB(tagBounds.Right, iconBounds.Top, iconBounds.Right, iconBounds.Bottom);

        IconDrawing.RenderScaledIcon(graphics, icon, iconBounds, offsetY: 1);

        if (Owner.Obsolete && CentralSettings.CanvasObsoleteTags)
            GH_GraphicsUtil.RenderObjectOverlay(graphics, Owner, iconBounds);
    }
}
