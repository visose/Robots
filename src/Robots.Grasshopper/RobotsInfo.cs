using System.Drawing;
using System.Reflection;
using Grasshopper.Kernel;

[assembly: GH_Loading(GH_LoadingDemand.ForceDirect)]

namespace Robots.Grasshopper
{
    public class RobotsInfo : GH_AssemblyInfo
    {
        public RobotsInfo() { }
        public override string Name => GetInfo<AssemblyProductAttribute>().Product;
        public override string AssemblyVersion => GetInfo<AssemblyInformationalVersionAttribute>().InformationalVersion;
        public override Bitmap Icon => Properties.Resources.iconRobot;
        public override string Description => GetInfo<AssemblyDescriptionAttribute>().Description;
        public override GH_LibraryLicense License => GH_LibraryLicense.opensource;
        public override string AuthorName => GetCompany()[0];
        public override string AuthorContact => GetCompany()[1];
        public override Guid Id => new("0c4dd17f-db66-4895-9565-412eb167503f");

        T GetInfo<T>() where T : Attribute
        {
            var assembly = Assembly.GetExecutingAssembly();
            return assembly.GetCustomAttribute<T>();
        }

        string[] GetCompany()
        {
            var company = GetInfo<AssemblyCompanyAttribute>().Company;
            return company.Split(new[] { " - " }, StringSplitOptions.None);
        }
    }
}