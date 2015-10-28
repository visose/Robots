using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using System;
using System.Collections.Generic;
using System.Linq;

namespace Robots.Grasshopper
{
    public class CreateProgram : GH_Component
    {
        public CreateProgram() : base("Create program", "Program", "Create a program", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("{5186EFD5-C042-4CA9-A7D2-E143F4848DEF}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddTextParameter("Name", "N", "Program name", GH_ParamAccess.item, "DefaultProgram");
            pManager.AddParameter(new RobotParameter(), "Robot", "R", "Robot", GH_ParamAccess.item);
            pManager.AddParameter(new CommandParameter(), "Init commands", "C", "Initialization commands", GH_ParamAccess.list);
            pManager.AddParameter(new TargetParameter(), "Targets", "T", "Targets", GH_ParamAccess.list);
            pManager[2].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("Code", "C", "Code", GH_ParamAccess.list);
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
            program.GenerateCode();
            DA.SetData(0, new GH_Program(program));
            DA.SetDataList(1, program.Code);
        }
    }

    public class SaveProgram : GH_Component
    {
        public SaveProgram() : base("Save program", "SaveProg", "Save a program", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.quinary;
        public override Guid ComponentGuid => new Guid("{1DE69EAA-AA4C-44F2-8748-F19B041F8F58}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("Folder", "F", "Folder", GH_ParamAccess.item);
            pManager[1].Optional = true;
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
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
        public CustomCode() : base("Custom code", "Custom", "Custom code", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("{FF997511-4A84-4426-AB62-AF94D19FF58F}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
            pManager.AddTextParameter("Code", "C", "Custom code", GH_ParamAccess.list);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;
            var code = new List<string>();

            if (!DA.GetData(0, ref program)) { return; }
            if (!DA.GetDataList(1, code)) { return; }
            program.Value.Code = code;

            DA.SetData(0, program);
        }
    }

    public class CheckProgram : GH_Component
    {
        public CheckProgram() : base("Check program", "CheckProg", "Check a program for errors", "Robots", "Components") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        public override Guid ComponentGuid => new Guid("{BE57878F-2E6E-46A3-9EB9-E78568B6DC6C}");
        protected override System.Drawing.Bitmap Icon => null;  // return Properties.Resources.visualstudio; 

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddParameter(new ProgramParameter(), "Program", "P", "Program", GH_ParamAccess.item);
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddTextParameter("Target index", "I", "Index of first target that contains errors", GH_ParamAccess.list);
            pManager.AddTextParameter("Errors", "E", "Errors in program", GH_ParamAccess.list);
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
            GH_Program program = null;

            if (!DA.GetData(0, ref program)) { return; }

            var errors = new List<string>();
            int i = program.Value.CheckKinematics(out errors);

            if (i != -1)
            {
                DA.SetData(0, new GH_Integer(i));
                this.AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, "Errors in program");
            }
            DA.SetDataList(1, errors);
        }
    }

    public class ProgramParameter : GH_PersistentParam<GH_Program>
    {
        public ProgramParameter() : base("Program", "Program", "This is a robot program", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        protected override System.Drawing.Bitmap Icon => null;   // 24x24 pixels // Properties.Resources.bitmapparameter;
        public override System.Guid ComponentGuid => new Guid("{9C4F1BB6-5FA2-44DA-B7EA-421AF31DA054}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Program value)
        {
            value = new GH_Program();
            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Program> values)
        {
            values = new List<GH_Program>();
            return GH_GetterResult.success;
        }
    }

    public class GH_Program : GH_Goo<Program>
    {
        public GH_Program() { this.Value = null; }
        public GH_Program(GH_Program goo) { this.Value = goo.Value; }
        public GH_Program(Program native) { this.Value = native; }
        public override IGH_Goo Duplicate() => new GH_Program(this);
        public override bool IsValid => true;
        public override string TypeName => "Program";
        public override string TypeDescription => "Program";
        public override string ToString() => this.Value.ToString();
    }
}