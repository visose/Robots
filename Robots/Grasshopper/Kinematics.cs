using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.GUI;
using Grasshopper;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;
using System.Drawing;

namespace Robots.Grasshopper
{
    public class Kinematics : GH_Component
    {
        public Kinematics() : base("Kinematics", "K", "Inverse and forward kinematics for a single target", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{EFDA05EB-B281-4703-9C9E-B5F98A9B2E1D}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconKinematics;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
            pManager.AddParameter(new TargetParameter(), "Target", "T", "Target", GH_ParamAccess.item);
            pManager.AddBooleanParameter("Geometry", "M", "Generate mesh geometry of the robot", GH_ParamAccess.item, false);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Robot meshes", "M", "Robot meshes", GH_ParamAccess.list);
            pManager.AddTextParameter("Joint rotations", "J", "Joint rotations", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Plane", "P", "Plane", GH_ParamAccess.item);
            pManager.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Robot robot = null;
            GH_Target target = new GH_Target();
            bool drawMeshes = false;

            if (!DA.GetData(0, ref robot)) { return; }
            if (!DA.GetData(1, ref target)) { return; }
            if (!DA.GetData(2, ref drawMeshes)) { return; }

            var kinematics = robot.Value.Kinematics(target.Value, drawMeshes);

            if (kinematics.Errors.Count > 0)
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Errors in solution");
            }

            var strings = kinematics.Joints.Select(x => new GH_Number(x).ToString());
            var joints = string.Join(",", strings);

            if (kinematics.Meshes != null) DA.SetDataList(0, kinematics.Meshes.Select(x => new GH_Mesh(x)));
            DA.SetData(1, joints);
            DA.SetData(2, kinematics.Planes[7]);
            DA.SetDataList(3, kinematics.Errors);
        }
    }

    public class DegreesToRadians : GH_Component
    {
        public DegreesToRadians() : base("Degrees to radians", "DegToRad", "Manufacturer dependent degrees to radians conversion.", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{C10B3A17-5C19-4805-ACCF-839B85C4D21C}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconAngles;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("Degrees", "D", "Degrees", GH_ParamAccess.list);
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Radians", "R", "Radians", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            List<double> degrees = new List<double>();
            GH_Robot robot = null;

            if (!DA.GetDataList(0, degrees)) { return; }
            if (!DA.GetData(1, ref robot)) { return; }

            var radians = degrees.Select((x, i) => robot.Value.DegreeToRadian(x, i));
            string radiansText = string.Join(",", radians.Select(x => $"{x:0.00}"));

            DA.SetData(0, radiansText);
        }
    }

    public class Simulation : GH_Component
    {
        public Simulation() : base("Program simulation", "Sim", "Rough simulation of the robot program, right click for playback controls", "Robots", "Components")
        {
            form = new AnimForm(this);
        }

        double time = 0;
        double sliderTime = 0;

        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{6CE35140-A625-4686-B8B3-B734D9A36CFC}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconSimulation;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddNumberParameter("Time", "T", "Advance the simulation to this time", GH_ParamAccess.item, 0);
            pManager.AddBooleanParameter("Normalized", "N", "Time value is normalized (from 0 to 1)", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Robot meshes", "M", "Robot meshes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Joint rotations", "J", "Joint rotations", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Planes", "P", "Planes", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Index", "I", "Current target index", GH_ParamAccess.item);
            pManager.AddNumberParameter("Time", "T", "Current time in seconds", GH_ParamAccess.item);
            pManager.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            GH_Number sliderTimeGH = null;
            GH_Boolean isNormalized = null;
            if (!DA.GetData(0, ref program)) { return; }
            if (!DA.GetData(1, ref sliderTimeGH)) { return; }
            if (!DA.GetData(2, ref isNormalized)) { return; }


            sliderTime = (isNormalized.Value) ? sliderTimeGH.Value * program.Value.Duration : sliderTimeGH.Value;
            if (!form.Visible) time = sliderTime;

            var kinematics = program.Value.Animate(time, false);

            if (kinematics.Errors.Count > 0)
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Errors in solution");

            DA.SetDataList(0, kinematics.Meshes.Select(x => new GH_Mesh(x)));
            DA.SetDataList(1, kinematics.Joints);
            DA.SetData(2, kinematics.Planes[7]);
            DA.SetData(3, program.Value.CurrentSimulationTarget.Index);
            DA.SetData(4, program.Value.CurrentSimulationTime);
            DA.SetDataList(5, kinematics.Errors);

            if (form.Visible && form.play.Checked)
            {
                var currentTime = DateTime.Now;
                TimeSpan delta = currentTime - lastTime;
                time += delta.TotalSeconds * speed;
                lastTime = currentTime;
                this.ExpireSolution(true);
            }
        }


        // Form
        AnimForm form;
        double speed = 1;
        DateTime lastTime;

        protected override void AppendAdditionalComponentMenuItems(ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Open controls", OpenForm, true, form.Visible);
        }

        void OpenForm(object sender, EventArgs e)
        {
            if (form.Visible)
                form.Hide();
            else
            {
                form.Show(Instances.DocumentEditor);
                GH_WindowsFormUtil.CenterFormOnCursor(form, true);
            }
        }

        void ClickPlay(object sender, EventArgs e)
        {
            lastTime = DateTime.Now;
            ExpireSolution(true);
        }

        void ClickStop(object sender, EventArgs e)
        {
            form.play.Checked = false;
            time = sliderTime;
            ExpireSolution(true);
        }

        void ClickScroll(object sender, EventArgs e)
        {
            double trackValue = form.slider.Value;

            if (form.slider.Value % form.slider.SmallChange != 0)
            {
                form.slider.Value = (form.slider.Value / form.slider.SmallChange) * form.slider.SmallChange;
            }

            speed = (double)form.slider.Value / 100.0;
        }

        partial class AnimForm : Form
        {
            Simulation component;

            internal CheckBox play = new CheckBox();
            private Button stop = new Button();
            internal TrackBar slider = new TrackBar();

            public AnimForm(Simulation component)
            {
                this.component = component;
                InitializeComponent();
            }

            protected override void OnFormClosing(FormClosingEventArgs e)
            {
                base.OnFormClosing(e);
                if (e.CloseReason == CloseReason.UserClosing)
                {
                    e.Cancel = true;
                    this.Hide();
                }
            }

            private System.ComponentModel.IContainer components = null;

            protected override void Dispose(bool disposing)
            {
                if (disposing && (components != null)) components.Dispose();
                base.Dispose(disposing);
            }

            private void InitializeComponent()
            {
                this.SuspendLayout();

                var controlFont = new Font(FontFamily.GenericSansSerif, 20f, FontStyle.Regular, GraphicsUnit.Pixel);
                var labelFont = new Font(FontFamily.GenericSansSerif, 10f, FontStyle.Regular, GraphicsUnit.Pixel);

                // Form
                Controls.Add(play);
                Controls.Add(stop);

                // AutoScaleDimensions = new SizeF(6F, 13F);
                AutoScaleMode = AutoScaleMode.None;
                AutoSize = false;
                MinimumSize = new Size(0, 0);
                Size = new Size(138, 320);
                Name = "Simulation controls";
                Text = "Simulation";
                ResumeLayout(false);
                FormBorderStyle = FormBorderStyle.FixedSingle;
                MinimizeBox = false;
                MaximizeBox = false;
                ShowIcon = false;
                TransparencyKey = Color.White;

                // Play
                play.Location = new Point(6, 6);
                play.Size = new Size(60, 60);
                play.Name = "Play";
                play.Text = "\u25B6";
                play.Font = controlFont;
                play.TextAlign = ContentAlignment.MiddleCenter;
                play.TabIndex = 0;
                play.UseVisualStyleBackColor = true;
                play.Click += new EventHandler(component.ClickPlay);
                play.Appearance = Appearance.Button;

                // Stop
                stop.Location = new Point(66, 6);
                stop.Size = new Size(60, 60);
                stop.Name = "Pause";
                stop.Text = "\u25FC";
                play.Font = controlFont;
                play.TextAlign = ContentAlignment.MiddleCenter;
                stop.TabIndex = 1;
                stop.UseVisualStyleBackColor = true;
                stop.Click += new EventHandler(component.ClickStop);


                // Slider group
                var group = new GroupBox();
                group.Location = new Point(6, 66);
                group.Size = new Size(120, 220);
                group.Text = "Speed";
                group.Font = labelFont;
                this.Controls.Add(group);

                // Slider
                slider.Location = new Point(6, 14);
                slider.Size = new Size(45, 200);
                slider.Orientation = Orientation.Vertical;
                slider.Name = "Speed";
                slider.TabIndex = 2;
                slider.Maximum = 400;
                slider.Minimum = -200;
                slider.TickFrequency = 100;
                slider.LargeChange = 100;
                slider.SmallChange = 50;
                slider.TickStyle = TickStyle.BottomRight;
                slider.Value = 100;
                slider.ValueChanged += new EventHandler(component.ClickScroll);
                group.Controls.Add(slider);

                // Slider labels
                int count = (slider.Maximum - slider.Minimum) / slider.TickFrequency;

                for (int i = 0; i <= count; i++)
                {
                    var label = new Label();

                    label.Location = new Point(51, 16 + i * 29);
                    label.Size = new Size(60, 20);
                    label.Text = (slider.Maximum - i * 100).ToString() + "%";
                    label.Font = labelFont;
                    label.TextAlign = ContentAlignment.MiddleLeft;
                    group.Controls.Add(label);
                }
            }
        }
    }

}
