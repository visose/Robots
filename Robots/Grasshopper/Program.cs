using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

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
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
            pManager.AddParameter(new CommandParameter(), "Init commands", "C", "Initialization commands", GH_ParamAccess.list);
            pManager.AddParameter(new TargetParameter(), "Targets", "T", "Targets", GH_ParamAccess.list);
            pManager[2].Optional = true;
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

            if (!DA.GetData(0, ref name)) { return; }
            if (!DA.GetData(1, ref robot)) { return; }
            DA.GetDataList(2, commands);
            if (!DA.GetDataList(3, targets)) { return; }

            var initCommand = new Robots.Commands.Group();
            initCommand.AddRange(commands.Select(x => x.Value));

            var program = new Program(name, robot.Value, targets.Select(x => x.Value).ToList(), initCommand);

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

}