using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Rhino.PlugIns;
using System;
using System.Drawing;
using Grasshopper.Kernel;


// General Information about an assembly is controlled through the following 
// set of attributes. Change these attribute values to modify the information
// associated with an assembly.
[assembly: AssemblyDescription("Provides components to visualize and create programs for ABB, KUKA and UR robots. Developed as an open source tool to interact with the robots at Bartlett School of Architecture.")]
[assembly: AssemblyCopyright("Copyright ©  2017")]
[assembly: AssemblyTrademark("")]
[assembly: AssemblyCulture("")]

// Setting ComVisible to false makes the types in this assembly not visible 
// to COM components.  If you need to access a type in this assembly from 
// COM, set the ComVisible attribute to true on that type.
[assembly: ComVisible(false)]

// The following GUID is for the ID of the typelib if this project is exposed to COM
[assembly: Guid("f6829dc0-c74c-4c3a-975a-684fc6dded73")] // This will also be the Guid of the Rhino plug-in

// Version information for an assembly consists of the following four values:
//
//      Major Version
//      Minor Version 
//      Build Number
//      Revision
//
// You can specify all the values or you can default the Build and Revision Numbers 
// by using the '*' as shown below:
// [assembly: AssemblyVersion("1.0.*")]

[assembly: GH_Loading(GH_LoadingDemand.ForceDirect)]

namespace Robots.Grasshopper
{
    public class RobotsInfo : GH_AssemblyInfo
    {
        public RobotsInfo() { }
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