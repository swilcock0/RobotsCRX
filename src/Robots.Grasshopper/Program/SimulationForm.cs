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
            MaxValue = Speeds.Length - 1,
            MinValue = 0,
            TickFrequency = 1,
            SnapToTick = true,
            Value = Array.IndexOf(Speeds, component.Speed),
        };

        Label speedLabel = new()
        {
            Text = Describe(component.Speed),
            VerticalAlignment = VerticalAlignment.Center,
        };

        slider.ValueChanged += (s, e) =>
        {
            component.Speed = Speeds[slider.Value];
            speedLabel.Text = Describe(component.Speed);
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

    // Doubling up to 128 times faster, and back in reverse.
    static readonly double[] Speeds = [-4, -2, -1, -0.5, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64, 128];

    static string Describe(double speed) => $"{speed:0.##}×";

    protected override void OnClosing(CancelEventArgs e)
    {
        _component.Stop();
        base.OnClosing(e);
    }
}
