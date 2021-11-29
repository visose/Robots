using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using System.Threading;
using System.Globalization;
using Rhino.Geometry;
using static Robots.Util;

namespace Robots
{
    public enum Manufacturers { ABB, KUKA, UR, FANUC, Staubli, Other, All };

    public abstract class RobotSystem
    {
        public string Name { get; }
        public Manufacturers Manufacturer { get; }
        public IO IO { get; }
        public Plane BasePlane { get; }
        public Mesh? Environment { get; }
        public Mesh DisplayMesh { get; } = new Mesh();
        public IRemote? Remote { get; protected set; }

        static RobotSystem()
        {
            CultureInfo.DefaultThreadCurrentCulture = CultureInfo.InvariantCulture;
            Thread.CurrentThread.CurrentCulture = CultureInfo.InvariantCulture;
        }

        protected RobotSystem(string name, Manufacturers manufacturer, IO io, Plane basePlane, Mesh? environment)
        {
            Name = name;
            Manufacturer = manufacturer;
            IO = io;
            BasePlane = basePlane;
            Environment = environment;
        }

        /// <summary>
        /// Quaternion interpolation based on: http://www.grasshopper3d.com/group/lobster/forum/topics/lobster-reloaded
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="t"></param>
        /// <param name="min"></param>
        /// <param name="max"></param>
        /// <returns></returns>
        public virtual Plane CartesianLerp(Plane a, Plane b, double t, double min, double max)
        {
            t = (t - min) / (max - min);
            if (double.IsNaN(t)) t = 0;
            var newOrigin = a.Origin * (1 - t) + b.Origin * t;

            //  Quaternion q = Quaternion.Rotation(a, b);
            // var q = Quaternion.Identity.Rotate(a).Rotate(b);

            var q = Slerp(GetRotation(a), GetRotation(b), t);

            //  q.GetRotation(out var angle, out var axis);
            // angle = (angle > PI) ? angle - 2 * PI : angle;
            //  a.Rotate(t * angle, axis, a.Origin);

            a = TransformFromQuaternion(q).ToPlane();

            a.Origin = newOrigin;
            return a;
        }

        internal abstract void SaveCode(IProgram program, string folder);
        internal abstract List<List<List<string>>> Code(Program program);
        internal abstract double Payload(int group);
        internal abstract Joint[] GetJoints(int group);
        public abstract List<KinematicSolution> Kinematics(IEnumerable<Target> target, IEnumerable<double[]>? prevJoints = null);
        public abstract double DegreeToRadian(double degree, int i, int group = 0);
        public abstract double[] PlaneToNumbers(Plane plane);
        public abstract Plane NumbersToPlane(double[] numbers);

        public static List<string> ListRobotSystems()
        {
            var names = new List<string>();
            var elements = new List<XElement>();
            //string folder = $@"{AssemblyDirectory}\robots";
            string folder = LibraryPath;

            if (Directory.Exists(folder))
            {
                var files = Directory.GetFiles(folder, "*.xml");
                foreach (var file in files)
                {
                    var element = XElement.Load(file);
                    if (element.Name.LocalName == "RobotSystems")
                        elements.AddRange(element.Elements());
                }
            }
            else
            {
                throw new DirectoryNotFoundException($" Folder '{folder}' not found");
            }

            //  elements.AddRange(XElement.Parse(Properties.Resources.robotsData).Elements());

            foreach (var element in elements)
                names.Add($"{element.Attribute(XName.Get("name")).Value}");

            return names;
        }

        public static RobotSystem Load(string name, Plane basePlane, bool loadMeshes = true)
        {
            XElement? element = null;
            //string folder = $@"{AssemblyDirectory}\robots";
            string folder = LibraryPath;

            if (Directory.Exists(folder))
            {
                var files = Directory.GetFiles(folder, "*.xml");

                foreach (var file in files)
                {
                    XElement data = XElement.Load(file);
                    if (data.Name.LocalName != "RobotSystems") continue;
                    element = data.Elements().FirstOrDefault(x => name == $"{x.Attribute(XName.Get("name")).Value}");
                    if (element != null)
                        break;
                }
            }
            else
            {
                throw new DirectoryNotFoundException($" Folder '{folder}' not found");
            }

            if (element is null)
                throw new InvalidOperationException($" RobotSystem \"{name}\" not found");

            /*
            if (element == null)
            {
                XElement data = XElement.Parse(Properties.Resources.robotsData);
                try
                {
                    element = data.Elements().First(x => name == $"{x.Attribute(XName.Get("name")).Value}");
                }
                catch (InvalidOperationException)
                {
                    throw new InvalidOperationException($" RobotSystem \"{name}\" not found");
                }
            }
            */

            return Create(element, basePlane, loadMeshes);
        }

        public static RobotSystem Parse(string xml, Plane basePlane)
        {
            var element = XElement.Parse(xml);
            return Create(element, basePlane, false);
        }

        private static RobotSystem Create(XElement element, Plane basePlane, bool loadMeshes)
        {
            var type = element.Name.LocalName;
            var name = element.Attribute(XName.Get("name")).Value;
            var manufacturer = (Manufacturers)Enum.Parse(typeof(Manufacturers), element.Attribute(XName.Get("manufacturer")).Value);
            //var mechanisms = new List<Mechanism>();

            var mechanicalGroups = new List<MechanicalGroup>();
            foreach (var mechanicalGroup in element.Elements(XName.Get("Mechanisms")))
            {
                mechanicalGroups.Add(MechanicalGroup.Create(mechanicalGroup, loadMeshes));
            }

            IO io;
            XElement ioElement = element.Element(XName.Get("IO"));

            if (ioElement != null)
            {
                string[]? doNames = null, diNames = null, aoNames = null, aiNames = null;

                var doElement = ioElement.Element(XName.Get("DO"));
                var diElement = ioElement.Element(XName.Get("DI"));
                var aoElement = ioElement.Element(XName.Get("AO"));
                var aiElement = ioElement.Element(XName.Get("AI"));

                if (doElement != null) doNames = doElement.Attribute(XName.Get("names")).Value.Split(',');
                if (diElement != null) diNames = diElement.Attribute(XName.Get("names")).Value.Split(',');
                if (aoElement != null) aoNames = aoElement.Attribute(XName.Get("names")).Value.Split(',');
                if (aiElement != null) aiNames = aiElement.Attribute(XName.Get("names")).Value.Split(',');

                io = new IO(doNames, diNames, aoNames, aiNames);
            }
            else
            {
                io = new IO();
            }

            Mesh? environment = null;

            if (type == "RobotCell")
            {
                return manufacturer switch
                {
                    Manufacturers.ABB => new RobotCellAbb(name, mechanicalGroups, io, basePlane, environment),
                    Manufacturers.KUKA => new RobotCellKuka(name, mechanicalGroups, io, basePlane, environment),
                    Manufacturers.UR => new RobotCellUR(name, (RobotUR)mechanicalGroups[0].Robot, io, basePlane, environment),
                    Manufacturers.FANUC => new RobotCellFanuc(name, mechanicalGroups, io, basePlane, environment),
                    Manufacturers.Staubli => new RobotCellStaubli(name, mechanicalGroups, io, basePlane, environment),
                    _ => throw new ArgumentException($" Manufacturer '{manufacturer} is not supported.")
                };
            }

            throw new ArgumentException($" Type '{type}' should be 'RobotCell'");
        }

        public override string ToString() => $"{GetType().Name} ({Name})";
    }

    public class IO
    {
        public string[] DO { get; }
        public string[] DI { get; }
        public string[] AO { get; }
        public string[] AI { get; }

        public IO(string[]? @do = null, string[]? di = null, string[]? ao = null, string[]? ai = null)
        {
            DO = @do ?? new string[0];
            DI = di ?? new string[0];
            AO = ao ?? new string[0];
            AI = ai ?? new string[0];
        }
    }
}
