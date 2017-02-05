using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Robots.Grasshopper
{
    public class RobotsInfo : GH_AssemblyInfo
    {
        public RobotsInfo()
        {
        }

        public override string Name => "Robots";
        public override Bitmap Icon => Properties.Resources.iconRobot;
        public override string Description => "Provides components to visualize and create programs for ABB, KUKA and UR robots. Developed as an open source tool to program the robots at Bartlett School of Architecture.";
        public override Guid Id => new Guid("0c4dd17f-db66-4895-9565-412eb167503f");
        public override string AssemblyVersion => "0.0.5";
        public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
        public override string AuthorName => "Bartlett BMADE";
        public override string AuthorContact => "v.soler@ucl.ac.uk, vincent.huyghe.13@ucl.ac.uk";
    }
}