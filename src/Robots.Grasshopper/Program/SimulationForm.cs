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
        MinimumSize = new(0, 200);

        Padding = new(5);

        Font font = new(FontFamilies.Sans, 14, FontStyle.None, FontDecoration.None);
        Size size = new(35, 35);

        Play = new CheckBox
        {
            Text = "\u25B6",
            Size = size,
            Font = font,
            Checked = false,
            TabIndex = 0
        };

        Play.CheckedChanged += (s, e) => component.TogglePlay();

        Button stop = new()
        {
            Text = "\u25FC",
            Size = size,
            Font = font,
            TabIndex = 1
        };

        stop.Click += (s, e) => component.Stop();

        Slider slider = new()
        {
            Orientation = Orientation.Vertical,
            Size = new(-1, -1),
            TabIndex = 2,
            MaxValue = 400,
            MinValue = -200,
            TickFrequency = 100,
            SnapToTick = true,
            Value = 100,
        };

        slider.ValueChanged += (s, e) => component.Speed = slider.Value / 100.0; ;

        Label speedLabel = new()
        {
            Text = "100%",
            VerticalAlignment = VerticalAlignment.Center,
        };

        DynamicLayout layout = new();
        _ = layout.BeginVertical();
        _ = layout.AddSeparateRow(padding: new(10), spacing: new(10, 0), controls: [Play, stop]);
        _ = layout.BeginGroup("Speed");
        _ = layout.AddSeparateRow(slider, speedLabel);
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
