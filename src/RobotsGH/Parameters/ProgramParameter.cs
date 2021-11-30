using System;
using System.Collections.Generic;
using Grasshopper.Kernel;

namespace Robots.Grasshopper
{
    public class ProgramParameter : GH_PersistentParam<GH_Program>
    {
        public ProgramParameter() : base("Program parameter", "Program", "This is a robot program", "Robots", "Parameters") { }
        public override GH_Exposure Exposure => GH_Exposure.tertiary;
        protected override System.Drawing.Bitmap Icon => Properties.Resources.iconProgramParam;
        public override Guid ComponentGuid => new Guid("{9C4F1BB6-5FA2-44DA-B7EA-421AF31DA054}");
        protected override GH_GetterResult Prompt_Singular(ref GH_Program value)
        {
            value = new GH_Program();

            if (value.Value is CustomProgram)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, " Program contains custom code");

            return GH_GetterResult.success;
        }
        protected override GH_GetterResult Prompt_Plural(ref List<GH_Program> values)
        {
            values = new List<GH_Program>();
            return GH_GetterResult.success;
        }
    }
}
