using System.Drawing;
using System.Windows.Forms;
using Grasshopper.Kernel.Attributes;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;

namespace Robots.Grasshopper;

class ComponentButton : GH_ComponentAttributes
{
    const int _buttonSize = 18;

    readonly string _label;
    readonly Action _action;

    RectangleF _buttonBounds;
    bool _mouseDown;

    public ComponentButton(GH_Component owner, string label, Action action) : base(owner)
    {
        _label = label;
        _action = action;
    }

    protected override void Layout()
    {
        base.Layout();

        const int margin = 3;

        var bounds = GH_Convert.ToRectangle(Bounds);
        var button = bounds;

        button.X += margin;
        button.Width -= margin * 2;
        button.Y = bounds.Bottom;
        button.Height = _buttonSize;

        bounds.Height += _buttonSize + margin;

        Bounds = bounds;
        _buttonBounds = button;
    }

    protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
    {
        base.Render(canvas, graphics, channel);

        if (channel == GH_CanvasChannel.Objects)
        {
            var prototype = GH_FontServer.StandardAdjusted;
            var font = GH_FontServer.NewFont(prototype, 6f / GH_GraphicsUtil.UiScale);
            var radius = 3;
            var highlight = !_mouseDown ? 8 : 0;

            using var button = GH_Capsule.CreateTextCapsule(_buttonBounds, _buttonBounds, GH_Palette.Black, _label, font, radius, highlight);
            button.Render(graphics, false, Owner.Locked, false);
        }
    }

    void SetMouseDown(bool value, GH_Canvas canvas, GH_CanvasMouseEvent e, bool action = true)
    {
        if (Owner.Locked || _mouseDown == value)
            return;

        if (value && e.Button != MouseButtons.Left)
            return;

        if (!_buttonBounds.Contains(e.CanvasLocation))
            return;

        if (_mouseDown && !value && action)
            _action();

        _mouseDown = value;
        canvas.Invalidate();
    }

    public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        SetMouseDown(true, sender, e);
        return base.RespondToMouseDown(sender, e);
    }

    public override GH_ObjectResponse RespondToMouseUp(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        SetMouseDown(false, sender, e);
        return base.RespondToMouseUp(sender, e);
    }

    public override GH_ObjectResponse RespondToMouseMove(GH_Canvas sender, GH_CanvasMouseEvent e)
    {
        SetMouseDown(false, sender, e, false);
        return base.RespondToMouseMove(sender, e);
    }
}