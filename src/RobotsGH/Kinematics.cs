using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Grasshopper.GUI;
using Grasshopper;
using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
//using System.Windows.Forms;
//using System.Drawing;
using Eto.Drawing;
using Eto.Forms;
using System.ComponentModel;

namespace Robots.Grasshopper
{
    public class Kinematics : GH_Component
    {
        public Kinematics() : base("Kinematics", "K", "Inverse and forward kinematics for a single target (or group of targets in a robot cell with coord", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{EFDA05EB-B281-4703-9C9E-B5F98A9B2E1D}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconKinematics;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new RobotSystemParameter(), "Robot system", "R", "Robot system", GH_ParamAccess.item);
            pManager.AddParameter(new TargetParameter(), "Target", "T", "One target per robot", GH_ParamAccess.list);
            pManager.AddTextParameter("Previous joints", "J", "Optional previous joint values. If the pose is ambigous is will select one based on this previous position.", GH_ParamAccess.list);
            pManager.AddBooleanParameter("Display geometry", "M", "Display mesh geometry of the robot", GH_ParamAccess.item, false);
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Meshes", "M", "Robot system's meshes", GH_ParamAccess.list);
            pManager.AddTextParameter("Joints", "J", "Robot system's joint rotations as a string of numbers separated by commas.", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Planes", "P", "Robot system's joint lanes", GH_ParamAccess.list);
            pManager.AddTextParameter("Errors", "E", "Errors in kinematic solution", GH_ParamAccess.list);
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

                        for (int i = 0; i < jointsText.Length; i++)
                            if (!GH_Convert.ToDouble_Secondary(jointsText[i], ref prevJoint[i])) throw new Exception(" Previous joints not formatted correctly.");

                        prevJoints.Add(prevJoint);
                    }
                }
            }

            var kinematics = robotSystem.Value.Kinematics(targets.Select(x => x.Value), prevJoints);

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
                var meshes = GeometryUtil.PoseMeshes(robotSystem.Value, kinematics, targets.Select(t => t.Value.Tool.Mesh).ToList());
                DA.SetDataList(0, meshes.Select(x => new GH_Mesh(x)));
            }

            DA.SetData(1, joints);
            DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
            DA.SetDataList(3, errors);
        }
    }

    public sealed class Simulation : GH_Component, IDisposable
    {
        public Simulation() : base("Program simulation", "Sim", "Rough simulation of the robot program, right click for playback controls", "Robots", "Components")
        {
            form = new AnimForm(this)
            {
                Owner = Rhino.UI.RhinoEtoApp.MainWindow
            };
        }

        double time = 0;
        double sliderTime = 0;

        public override GH_Exposure Exposure => GH_Exposure.quarternary;
        public override Guid ComponentGuid => new Guid("{6CE35140-A625-4686-B8B3-B734D9A36CFC}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconSimulation;

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
            var currentTarget = program.Value.CurrentSimulationTarget;

            var errors = currentTarget.ProgramTargets.SelectMany(x => x.Kinematics.Errors);
            var joints = currentTarget.ProgramTargets.SelectMany(x => x.Kinematics.Joints);
            var planes = currentTarget.ProgramTargets.SelectMany(x => x.Kinematics.Planes).ToList();
            var meshes = GeometryUtil.PoseMeshes(program.Value.RobotSystem, currentTarget.ProgramTargets.Select(p => p.Kinematics).ToList(), currentTarget.ProgramTargets.Select(p => p.Target.Tool.Mesh).ToList());

            if (errors.Count() > 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in solution");

            DA.SetDataList(0, meshes);
            DA.SetDataList(1, joints);
            DA.SetDataList(2, planes.Select(x => new GH_Plane(x)));
            DA.SetData(3, currentTarget.Index);
            DA.SetData(4, program.Value.CurrentSimulationTime);
            DA.SetData(5, new GH_Program(program.Value));
            DA.SetDataList(6, errors);

            if (form.Visible && form.play.Checked.Value)
            {
                var currentTime = DateTime.Now;
                TimeSpan delta = currentTime - lastTime;
                time += delta.TotalSeconds * speed;
                lastTime = currentTime;
                ExpireSolution(true);
            }
        }

        // Form
        AnimForm form;
        double speed = 1;
        DateTime lastTime;

        protected override void AppendAdditionalComponentMenuItems(System.Windows.Forms.ToolStripDropDown menu)
        {
            Menu_AppendItem(menu, "Open controls", OpenForm, true, form.Visible);
        }

        void OpenForm(object sender, EventArgs e)
        {
            if (form.Visible)
            {
                form.play.Checked = false;
                form.Visible = false;
            }
            else
            {
                var mousePos = Mouse.Position;
                int x = (int)mousePos.X + 20;
                int y = (int)mousePos.Y - 160;

                form.Location = new Eto.Drawing.Point(x, y);
                form.Show();
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
            speed = (double)form.slider.Value / 100.0;
        }

        public void Dispose()
        {
            form.Dispose();
        }

        class AnimForm : Form
        {
            Simulation _component;

            internal CheckBox play;
            internal Slider slider;

            public AnimForm(Simulation component)
            {
                _component = component;

                Maximizable = false;
                Minimizable = false;
                Padding = new Padding(5);
                Resizable = false;
                ShowInTaskbar = true;
                Topmost = true;
                Title = "Playback";
                WindowStyle = WindowStyle.Default;

                var font = new Font(FontFamilies.Sans, 12, FontStyle.None, FontDecoration.None);
                var size = new Size(35, 35);

                play = new CheckBox()
                {
                    Text = "\u25B6",
                    Size = size,
                    Font = font,
                    Checked = false,
                    TabIndex = 0
                };
                play.CheckedChanged += component.ClickPlay;

                var stop = new Button()
                {
                    Text = "\u25FC",
                    Size = size,
                    Font = font,
                    TabIndex = 1
                };
                stop.Click += component.ClickStop;

                slider = new Slider()
                {
                    Orientation = Orientation.Vertical,
                    Size = new Size(45, 200),
                    TabIndex = 2,
                    MaxValue = 400,
                    MinValue = -200,
                    TickFrequency = 100,
                    SnapToTick = true,
                    Value = 100,
                };
                slider.ValueChanged += _component.ClickScroll;

                var speedLabel = new Label()
                {
                    Text = "100%",
                    VerticalAlignment = VerticalAlignment.Center,
                };

                var layout = new DynamicLayout();
                layout.BeginVertical(new Padding(2), Size.Empty);
                layout.AddSeparateRow(padding: new Padding(10), spacing: new Size(10, 0), controls: new Control[] { play, stop });
                layout.BeginGroup("Speeds");
                layout.AddSeparateRow(slider, speedLabel);
                layout.EndGroup();
                layout.EndVertical();

                Content = layout;
            }

            protected override void OnClosing(CancelEventArgs e)
            {
                //base.OnClosing(e);
                e.Cancel = true;
                play.Checked = false;
                Visible = false;
            }
        }
    }

}
