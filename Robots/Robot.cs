using System;
using System.Xml.Linq;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using static System.Math;

using Rhino.Geometry;
using static Robots.Util;
using static Rhino.RhinoMath;
using System.Runtime.Serialization.Formatters.Binary;
using System.IO;

namespace Robots
{
    [Serializable]
    public abstract partial class Robot
    {
        public enum Manufacturer { ABB, KUKA, UR };

        readonly string model;
        protected Manufacturer manufacturer;
        public string Model { get { return $"{manufacturer.ToString()}.{model}"; } }
        public string Extension {get ;}

        readonly double l2, ad2;
        Plane basePlane;
        readonly Mesh baseMesh;
        readonly Joint[] joints;

        public Robot(string model, Manufacturer manufacturer, string extension, Plane basePlane, Mesh baseMesh, Joint[] joints)
        {
            this.model = model;
            this.manufacturer = manufacturer;
            this.Extension = extension;
            this.basePlane = basePlane;
            this.baseMesh = baseMesh;
            this.joints = joints;

            l2 = Sqrt(joints[2].A * joints[2].A + joints[3].D * joints[3].D);
            ad2 = Atan2(joints[2].A, joints[3].D);
            var kinematics = Kinematics(new Target(new double[] { 0, PI / 2, 0, 0, 0, -PI }), false);
            for (int i = 0; i < 6; i++) joints[i].Plane = kinematics.JointPlanes[i];
        }

        public static Robot Create(string model, Manufacturer manufacturer, Plane basePlane, double[] a, double []d, Interval[] range, double[] maxSpeed, Mesh[] mesh)
        {
            Joint[] joints = new Joint[6];
            for (int i = 0; i < 6; i++)
            {
                joints[i] = new Joint(a[i], d[i], range[i], maxSpeed[i], mesh[i+1]);
            }

            switch (manufacturer)
            {
                case (Manufacturer.ABB):
                    return null;
                case (Manufacturer.KUKA):
                    return new RobotKUKA(model, basePlane, mesh[0], joints);
                case (Manufacturer.UR):
                    return null;
                default:
                    return null;
            }
        }
       
        public static Robot Load(string model, Plane basePlane)
        {
           using (var stream = new MemoryStream(Properties.Resources.Robots))
           {
               var formatter = new BinaryFormatter();
               List<Robot> robots = formatter.Deserialize(stream) as List<Robot>;
               var robot = robots.First(x => x.Model == model);
               robot.basePlane = Plane.WorldXY;
               return robot;
           }
        }

        public static void Write()
        {
            string folder = @"C:\Users\Vicente\Documents\Trabajo\Barlett\RobotsApp\Robots\Robots\Resources";
            var robots = new List<Robot>();
            XElement robotsData = XElement.Load($@"{folder}\robotsData.xml");
            Rhino.FileIO.File3dm robotsGeometry = Rhino.FileIO.File3dm.Read($@"{folder}\robotsGeometry.3dm");

            foreach (var robotData in robotsData.Elements())
            {
                var model = robotData.Attribute(XName.Get("model")).Value;
                var manufacturer = (Manufacturer)Enum.Parse(typeof(Manufacturer), robotData.Attribute(XName.Get("manufacturer")).Value);

                var robotLayer = robotsGeometry.Layers.First(x => x.Name == model);
                Mesh baseMesh = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == robotLayer.LayerIndex).Geometry as Mesh;

                XElement[] elements = robotData.Elements().ToArray();
                Joint[] joints = new Joint[6];
                double[] t = new double[] { 1, 1, 1, 1, 1, 1 };
                var radians = DegreesToRadians(t);

                for (int i = 0; i < 6; i++)
                {
                    double a = Convert.ToDouble(elements[i].Attribute(XName.Get("a")).Value);
                    double d = Convert.ToDouble(elements[i].Attribute(XName.Get("d")).Value);
                    double minRange = Convert.ToDouble(elements[i].Attribute(XName.Get("minrange")).Value);
                    double maxRange = Convert.ToDouble(elements[i].Attribute(XName.Get("maxrange")).Value);
                    Interval range = new Interval(DegreeToRadian(minRange, i), DegreeToRadian(maxRange, i));
                    double maxSpeed = Convert.ToDouble(elements[i].Attribute(XName.Get("maxspeed")).Value);

                    //  string layerPath = $"{model}${i + 1:0}";
                    string layerPath = $"{(i + 1):0}";
                    var jointLayer = robotsGeometry.Layers.First(x => x.FullPath == layerPath);
                    Mesh mesh = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == jointLayer.LayerIndex).Geometry as Mesh;

                    joints[i] = new Joint(a, d, new Interval(minRange, maxRange), maxSpeed, mesh);
                }

                Robot robot = null;

                switch (manufacturer)
                {
                    case (Manufacturer.ABB):
                        robot = null;
                        break;
                    case (Manufacturer.KUKA):
                        robot = new RobotKUKA(model, Plane.WorldXY, baseMesh, joints);
                        break;
                    case (Manufacturer.UR):
                        robot = null;
                        break;
                }
                robots.Add(robot);
            }

            using (var stream = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(stream, robots);
                File.WriteAllBytes($@"{folder}\Robots.rob", stream.ToArray());
            }
        }

        public IKinematics Kinematics(Target target, bool calculateMeshes = true)
        {
            return new SphericalWristKinematics(target, this, calculateMeshes);
        }

        public override string ToString()
        {
            return $"Robot {manufacturer.ToString()} {model}";
        }

        abstract public StringBuilder Code(Program program);

        [Serializable]
        public class Joint
        {
            public double A { get; }
            public double D { get; }
            public Interval Range { get; }
            public double MaxSpeed { get; }
            public Plane Plane { get; set; }
            public Mesh Mesh { get; }

            public Joint(double a, double d, Interval range, double maxSpeed, Mesh mesh)
            {
                this.A = a;
                this.D = d;
                this.Range = range;
                this.MaxSpeed = maxSpeed;
                this.Mesh = mesh;
            }
        }
    }
}