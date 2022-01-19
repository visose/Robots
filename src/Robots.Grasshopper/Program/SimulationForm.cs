using System.ComponentModel;
using Eto.Drawing;
using Eto.Forms;

namespace Robots.Grasshopper;

class SimulationForm : ComponentForm
{
    readonly Simulation _component;

    internal readonly CheckBox Play;

    public SimulationForm(Simulation component)
    {
        _component = component;

        Title = "Playback";
        MinimumSize = new Size(0, 200);

        Padding = new Padding(5);

        var font = new Font(FontFamilies.Sans, 14, FontStyle.None, FontDecoration.None);
        var size = new Size(35, 35);

        Play = new CheckBox
        {
            Text = "\u25B6",
            Size = size,
            Font = font,
            Checked = false,
            TabIndex = 0
        };

        Play.CheckedChanged += (s, e) => component.TogglePlay();

        var stop = new Button
        {
            Text = "\u25FC",
            Size = size,
            Font = font,
            TabIndex = 1
        };

        stop.Click += (s, e) => component.Stop();

        var slider = new Slider
        {
            Orientation = Orientation.Vertical,
            Size = new Size(-1, -1),
            TabIndex = 2,
            MaxValue = 400,
            MinValue = -200,
            TickFrequency = 100,
            SnapToTick = true,
            Value = 100,
        };

        slider.ValueChanged += (s, e) => component.Speed = (double)slider.Value / 100.0; ;

        var speedLabel = new Label
        {
            Text = "100%",
            VerticalAlignment = VerticalAlignment.Center,
        };

        var layout = new DynamicLayout();
        layout.BeginVertical();
        layout.AddSeparateRow(padding: new Padding(10), spacing: new Size(10, 0), controls: new Control[] { Play, stop });
        layout.BeginGroup("Speed");
        layout.AddSeparateRow(slider, speedLabel);
        layout.EndGroup();
        layout.EndVertical();

        Content = layout;
    }

    protected override void OnClosing(CancelEventArgs e)
    {
        _component.Stop();
        base.OnClosing(e);
    }
}