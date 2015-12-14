using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;
using static System.Math;

namespace Robots.Grasshopper
{
    public class CreateProgram : GH_Component
    {
        public CreateProgram() : base("Create program", "Program", "Creates a program, checks for possible issues and fixes common mistakes", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("{5186EFD5-C042-4CA9-A7D2-E143F4848DEF}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCreateProgram;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Program name", GH_ParamAccess.item, "DefaultProgram");
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot used in program", GH_ParamAccess.item);
            pManager.AddParameter(new TargetParameter(), "Targets", "T", "List of targets", GH_ParamAccess.list);
            pManager.AddParameter(new CommandParameter(), "Init commands", "C", "Optional list of commands that will run at the start of the program", GH_ParamAccess.list);
            pManager.AddNumberParameter("Step Size", "S", "Distance in mm to step through linear motions, used for error checking and program simulation. Smaller is more accurate but slower", GH_ParamAccess.item, 1);
            pManager[3].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("Code", "C", "Code", GH_ParamAccess.list);
            pManager.AddNumberParameter("Duration", "D", "Program duration in seconds", GH_ParamAccess.item);
            pManager.AddTextParameter("Warnings", "W", "Warnings in program", GH_ParamAccess.list);
            pManager.AddTextParameter("Errors", "E", "Errors in program", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string name = null;
            GH_Robot robot = null;
            var commands = new List<GH_Command>();
            var targets = new List<GH_Target>();
            double stepSize = 1;

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref robot)) { return; }
            if (!DA.GetDataList(2, targets)) { return; }
            DA.GetDataList(3, commands);
            if (!DA.GetData(4, ref stepSize)) { return; }

            var initCommand = new Robots.Commands.Group();
            initCommand.AddRange(commands.Select(x => x.Value));

            var program = new Program(name, robot.Value, targets.Select(x => x.Value).ToList(), initCommand, stepSize);

            DA.SetData(0, new GH_Program(program));
            DA.SetDataList(1, program.Code);
            DA.SetData(2, program.Duration);

            if (program.Warnings.Count > 0)
            {
                DA.SetDataList(3, program.Warnings);
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Warnings in program");
            }

            if (program.Errors.Count > 0)
            {
                DA.SetDataList(4, program.Errors);
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Errors in program");
            }
        }
    }

    public class SaveProgram : GH_Component
    {
        public SaveProgram() : base("Save program", "SaveProg", "Saves a program to a text file", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quinary;
        public override Guid ComponentGuid => new Guid("{1DE69EAA-AA4C-44F2-8748-F19B041F8F58}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconSave;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("Folder", "F", "Folder", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            var code = new List<string>();
            string folder = null;


            if (!DA.GetData(0, ref program)) { return; }
            if (!DA.GetData(1, ref folder)) { return; }
            program.Value.Save(folder);
        }
    }

    public class CustomCode : GH_Component
    {
        public CustomCode() : base("Custom code", "Custom", "Creates a program using manufacturer specific custom code. This program cannot be simulated", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("{FF997511-4A84-4426-AB62-AF94D19FF58F}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCustomCode;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("Code", "C", "Custom code", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            var code = new List<string>();

            if (!DA.GetData(0, ref program)) { return; }
            if (!DA.GetDataList(1, code)) { return; }
            var newProgram = program.Value.CustomCode(code);
            DA.SetData(0, new GH_Program(newProgram));
        }
    }

    public class CheckCollisions : GH_Component
    {
        public CheckCollisions() : base("Check collisions", "Collisions", "Checks for possible collisions", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("{2848F557-8DF4-415A-800B-261E782E92F8}");
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconCollision;

        protected override void RegisterInputParams(GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddIntegerParameter("First set", "A", "First set of objects (0 = base, 1-6 = axis, 7 = tool)", GH_ParamAccess.list, new int[] { 7 });
            pManager.AddIntegerParameter("Second set", "B", "Second set of objects (0 = base, 1-6 = axis, 7 = tool)", GH_ParamAccess.list, new int[] { 4 });
            pManager.AddNumberParameter("Linear step size", "Ls", "Linear step size in mm to check for collisions", GH_ParamAccess.item, 100);
            pManager.AddNumberParameter("Angular step size", "As", "Angular step size in rad to check for collisions", GH_ParamAccess.item, PI / 4);
        }

        protected override void RegisterOutputParams(GH_OutputParamManager pManager)
        {
            pManager.AddBooleanParameter("Collision found", "C", "True if a collision was found", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Target index", "I", "Index of the first target where a collision was found (targets are not necessarily calculated in order)", GH_ParamAccess.item);
            pManager.AddMeshParameter("Collided meshes", "M", "Meshes involved in the collision", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            var first = new List<GH_Integer>();
            var second = new List<GH_Integer>();
            double linearStep = 100;
            double angularStep = PI / 4;

            if (!DA.GetData(0, ref program)) { return; }
            if (!DA.GetDataList(1, first)) { return; }
            if (!DA.GetDataList(2, second)) { return; }
            if (!DA.GetData(3, ref linearStep)) { return; }
            if (!DA.GetData(4, ref angularStep)) { return; }

            var collision = program.Value.CheckCollisions(first.Select(x => x.Value), second.Select(x => x.Value), linearStep, angularStep);

            DA.SetData(0, collision.HasCollision);
            if (collision.HasCollision) DA.SetData(1, collision.Target.Index);
            if (collision.HasCollision) DA.SetDataList(2, collision.Meshes);
        }
    }

}