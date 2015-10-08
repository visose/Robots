using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Robots.Grasshopper
{
    public class RobotsInfo : GH_AssemblyInfo
    {
        public override string Name => "Robots";
        public override Bitmap Icon => null;
        public override string Description => "";
        public override Guid Id => new Guid("0c4dd17f-db66-4895-9565-412eb167503f");

        public override string AuthorName => "";
        public override string AuthorContact => "";
    }
}
