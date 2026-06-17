using System.Drawing;

using Grasshopper;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Grasshopper.Kernel.Attributes;

namespace Robots.Grasshopper;

class ParameterAttributes(IGH_Param owner) : GH_FloatingParamAttributes(owner)
{
    GH_StateTagList? _stateTags;
    Rectangle _iconBounds;

    protected override void Layout()
    {
        base.Layout();

        _iconBounds = GH_Convert.ToRectangle(new RectangleF(
            Pivot.X - DefaultWidth * 0.5f,
            Pivot.Y - IconHeight * 0.5f,
            DefaultWidth,
            IconHeight));

        _stateTags = Owner.StateTags;

        if (_stateTags.Count == 0)
        {
            _stateTags = null;
            return;
        }

        _stateTags.Layout(_iconBounds, GH_StateTagLayoutDirection.Left);
    }

    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        if (channel != GH_CanvasChannel.Objects || !IsIconMode(Owner.IconDisplayMode))
        {
            base.Render(canvas, graphics, channel);
            return;
        }

        RenderCapsule(canvas, graphics);
    }

    void RenderCapsule(GH_Canvas canvas, Graphics graphics)
    {
        var bounds = Bounds;

        if (!canvas.Viewport.IsVisible(ref bounds, 10))
            return;

        Bounds = bounds;

        var hidden = Owner is not IGH_PreviewObject { IsPreviewCapable: true } preview || preview.Hidden;
        using var capsule = GH_Capsule.CreateCapsule(Bounds, GH_CapsuleRenderEngine.GetImpliedPalette(Owner));

        if (HasInputGrip)
            capsule.AddInputGrip(InputGrip.Y);

        if (HasOutputGrip)
            capsule.AddOutputGrip(OutputGrip.Y);

        capsule.Render(graphics, Selected, Owner.Locked, hidden);

        RenderIcon(graphics, _iconBounds);

        _stateTags?.RenderStateTags(graphics);
    }

    void RenderIcon(Graphics graphics, RectangleF iconBounds)
    {
        var icon = Owner.Locked
            ? Owner.Icon_24x24_Locked ?? Owner.Icon_24x24
            : Owner.Icon_24x24;

        if (icon is null || GH_Canvas.ZoomFadeLow < 5)
            return;

        IconDrawing.RenderScaledIcon(graphics, icon, iconBounds, offsetY: 1);

        if (Owner.Obsolete && CentralSettings.CanvasObsoleteTags)
            GH_GraphicsUtil.RenderObjectOverlay(graphics, Owner, iconBounds);
    }
}
