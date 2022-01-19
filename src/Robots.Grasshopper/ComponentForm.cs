using System.ComponentModel;
using Eto.Drawing;
using Eto.Forms;

namespace Robots.Grasshopper;

class ComponentForm : Form
{
    public ComponentForm()
    {
        Maximizable = false;
        Minimizable = false;
        Resizable = false;
        Topmost = true;
        ShowInTaskbar = true;
        Owner = Rhino.UI.RhinoEtoApp.MainWindow;
    }

    public override bool Visible
    {
        get => base.Visible;
        set
        {
            if (value)
                CenterOnMouse();

            base.Visible = value;
        }
    }

    void CenterOnMouse()
    {
        var mousePos = Mouse.Position;
        int x = (int)mousePos.X + 20;
        int y = (int)mousePos.Y - MinimumSize.Height / 2;
        Location = new Point(x, y);
    }

    protected override void OnClosing(CancelEventArgs e)
    {
        Visible = false;
        e.Cancel = true;

        base.OnClosing(e);
    }
}