using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.GUI;
using Grasshopper;
using Rhino.Geometry;
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
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
            pManager.AddParameter(new TargetParameter(), "Target", "T", "One target per robot", GH_ParamAccess.list);
            pManager.AddTextParameter("PrevJoints", "J", "Optional previous joint values. If the pose is ambigous is will select one based on this previous position.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Geometry", "M", "Generate mesh geometry of the robot", GH_ParamAccess.item, false);
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mechanism meshes", "M", "Mechanism meshes", GH_ParamAccess.list);
            pManager.AddTextParameter("Joint rotations", "J", "Joint rotations", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Plane", "P", "Plane", GH_ParamAccess.item);
            pManager.AddTextParameter("Errors", "E", "Errors", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_RobotSystem robotSystem = null;
            var targets = new List<GH_Target>();
            var prevJointsText = new List<GH_String>();
            bool drawMeshes = false;

            if (!DA.GetData(0, ref robotSystem)) { return; }
            if (!DA.GetDataList(1, targets)) { return; }
            DA.GetDataList(2, prevJointsText);
            if (!DA.GetData(3, ref drawMeshes)) { return; }

            List<double[]> prevJoints = null;

            if (prevJointsText.Count > 0)
            {
                prevJoints = new List<double[]>();

                foreach (var text in prevJointsText)
                {
                    if (text != null)
                    {
                        string[] jointsText = text.Value.Split(',');
                        var prevJoint = new double[jointsText.Length];

                        for (int i = 0; i < 6; i++)
                            if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref prevJoint[i])) return;

                        prevJoints.Add(prevJoint);
                    }
                }
            }

            var kinematics = robotSystem.Value.Kinematics(targets.Select(x => x.Value).ToList(), prevJoints, drawMeshes);

            var errors = kinematics.SelectMany(x => x.Errors);
            if (errors.Count() > 0)
            {
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");
            }

            var strings = kinematics.SelectMany(x => x.Joints).Select(x => new GH_Number(x).ToString());
            var joints = string.Join(",", strings);

            var planes = kinematics.SelectMany(x => x.Planes);
            if (drawMeshes)
            {
                var meshes = kinematics.SelectMany(x => x.Meshes);
                DA.SetDataList(0, meshes.Select(x => new GH_Mesh(x)));
            }

            DA.SetData(1, joints);
            DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
            DA.SetDataList(3, errors);
        }
    }

    public class Simulation : GH_Component, IDisposable
    {
        public Simulation() : base("Program simulation", "Sim", "Rough simulation of the robot program, right click for playback controls", "Robots", "Components")
        {
            form = new AnimForm(this);
        }

        double time = 0;
        double sliderTime = 0;

        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{6CE35140-A625-4686-B8B3-B734D9A36CFC}");
        protected override Bitmap Icon => Properties.Resources.iconSimulation;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program to simulate", GH_ParamAccess.item);
            pManager.AddNumberParameter("Time", "T", "Advance the simulation to this time", GH_ParamAccess.item, 0);
            pManager.AddBooleanParameter("Normalized", "N", "Time value is normalized (from 0 to 1)", GH_ParamAccess.item, true);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("System meshes", "M", "System meshes", GH_ParamAccess.list);
            pManager.AddNumberParameter("Joint rotations", "J", "Joint rotations", GH_ParamAccess.list);
            pManager.AddPlaneParameter("Plane", "P", "TCP position", GH_ParamAccess.list);
            pManager.AddIntegerParameter("Index", "I", "Current target index", GH_ParamAccess.item);
            pManager.AddNumberParameter("Time", "T", "Current time in seconds", GH_ParamAccess.item);
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "This is the same program as the input program. Use this output to update other visualization components along with the simulation.", GH_ParamAccess.item);
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

            program.Value.Animate(time, false);
            var kinematics = program.Value.CurrentSimulationKinematics;

            var errors = kinematics.SelectMany(x => x.Errors);
            var meshes = kinematics.SelectMany(x => x.Meshes);
            var joints = kinematics.SelectMany(x => x.Joints);
            var planes = kinematics.SelectMany(x => x.Planes);

            if (errors.Count() > 0)
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");

            DA.SetDataList(0, meshes.Select(x => new GH_Mesh(x)));
            DA.SetDataList(1, joints);
            DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
            DA.SetData(3, program.Value.CurrentSimulationTargets[0].Index);
            DA.SetData(4, program.Value.CurrentSimulationTime);
            DA.SetData(5, new GH_Program(program.Value));
            DA.SetDataList(6, errors);

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

        public void Dispose()
        {
            form.Dispose();
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
                play.Location = new System.Drawing.Point(6, 6);
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
                stop.Location = new System.Drawing.Point(66, 6);
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
                group.Location = new System.Drawing.Point(6, 66);
                group.Size = new Size(120, 220);
                group.Text = "Speed";
                group.Font = labelFont;
                this.Controls.Add(group);

                // Slider
                slider.Location = new System.Drawing.Point(6, 14);
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

                    label.Location = new System.Drawing.Point(51, 16 + i * 29);
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
