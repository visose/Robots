using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Runtime.Serialization.Formatters.Binary;
using System.Xml.Linq;
using static Robots.Util;
using static System.Math;

namespace Robots
{

    public abstract partial class Robot
    {
        public enum Manufacturers { ABB, KUKA, UR };

        readonly string model;
        public Manufacturers Manufacturer { get; protected set; }
        public string Model => $"{Manufacturer.ToString()}.{model}";
        internal string Extension { get; set; }
        public RobotIO IO { get; }
        protected Plane basePlane;
        readonly Mesh baseMesh;
        readonly Joint[] joints;

        public Mesh[] GetMeshes()
        {
            return joints.Select(x => x.Mesh).ToArray();
        }

        public Plane[] GetPlanes()
        {
            return joints.Select(x => x.Plane).ToArray();
        }


        internal Robot(string model, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io)
        {
            this.model = model;
            this.basePlane = basePlane;
            this.baseMesh = baseMesh;
            this.joints = joints;
            this.IO = io;

            for (int i = 0; i < 6; i++)
                joints[i].Range = new Interval(DegreeToRadian(joints[i].Range.T0, i), DegreeToRadian(joints[i].Range.T1, i));

            var kinematics = Kinematics(new Target(GetStartPose()));
            for (int i = 0; i < 6; i++)
                joints[i].Plane = kinematics.Planes[i + 1];
        }

        public static List<string> ListLibrary()
        {
            var names = new List<string>();
            XElement robotsData = XElement.Parse(Properties.Resources.robotsData);

            foreach (var element in robotsData.Elements())
            {
                names.Add($"{element.Attribute(XName.Get("manufacturer")).Value}.{element.Attribute(XName.Get("model")).Value}");
            }
            return names;
        }

        public static Robot LoadFromLibrary(string model, Plane basePlane)
        {
            XElement robotsData = XElement.Parse(Properties.Resources.robotsData);
            XElement robotElement = null;
            try
            {
                robotElement = robotsData.Elements().First(x => model == $"{x.Attribute(XName.Get("manufacturer")).Value}.{x.Attribute(XName.Get("model")).Value}");
            }
            catch (InvalidOperationException)
            {

                throw new InvalidOperationException($" Robot \"{model}\" is not in the library");
            }

            Mesh[] meshes;
            using (var stream = new MemoryStream(Properties.Resources.Meshes))
            {
                var formatter = new BinaryFormatter();
                RobotMeshes robotMeshes = formatter.Deserialize(stream) as RobotMeshes;
                int index = robotMeshes.Names.FindIndex(x => x == model);
                meshes = robotMeshes.Meshes[index];
            }

            return Load(robotElement, meshes, basePlane);
        }

        public static Robot LoadFromFile(string model, Plane basePlane)
        {
            string folder = AppDomain.CurrentDomain.RelativeSearchPath;

            XElement robotsData = XElement.Load($@"{folder}\robotsData.xml");
            XElement robotElement = null;
            try
            {
                robotElement = robotsData.Elements().First(x => model == $"{x.Attribute(XName.Get("manufacturer")).Value}.{x.Attribute(XName.Get("model")).Value}");
            }
            catch (InvalidOperationException)
            {

                throw new InvalidOperationException($" Robot \"{model}\" is not in the file");
            }

            Rhino.FileIO.File3dm robotsGeometry = Rhino.FileIO.File3dm.Read($@"{folder}\robotsGeometry.3dm");
            var meshes = new Mesh[7];

            var robotLayer = robotsGeometry.Layers.First(x => x.Name == $"{model}");
            meshes[0] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == robotLayer.LayerIndex).Geometry as Mesh;

            for (int i = 0; i < 6; i++)
            {
                meshes[i + 1] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == robotLayer.LayerIndex + 1 + i).Geometry as Mesh;
            }

            return Load(robotElement, meshes, basePlane);
        }

        private static Robot Load(XElement robotElement, Mesh[] meshes, Plane basePlane)
        {
            var modelName = robotElement.Attribute(XName.Get("model")).Value;
            var manufacturer = (Manufacturers)Enum.Parse(typeof(Manufacturers), robotElement.Attribute(XName.Get("manufacturer")).Value);
            var baseMesh = meshes[0].DuplicateMesh();

            var jointElements = robotElement.Element(XName.Get("Joints")).Descendants().ToArray();
            Joint[] joints = new Joint[6];

            for (int i = 0; i < 6; i++)
            {
                var jointElement = jointElements[i];
                double a = Convert.ToDouble(jointElement.Attribute(XName.Get("a")).Value);
                double d = Convert.ToDouble(jointElement.Attribute(XName.Get("d")).Value);
                double minRange = Convert.ToDouble(jointElement.Attribute(XName.Get("minrange")).Value);
                double maxRange = Convert.ToDouble(jointElement.Attribute(XName.Get("maxrange")).Value);
                Interval range = new Interval(minRange, maxRange);
                double maxSpeed = Convert.ToDouble(jointElement.Attribute(XName.Get("maxspeed")).Value);
                Mesh mesh = meshes[i + 1].DuplicateMesh();

                joints[i] = new Joint() { A = a, D = d, Range = range, MaxSpeed = maxSpeed, Mesh = mesh };
            }

            var ioElement = robotElement.Element(XName.Get("IO"));
            string[] doNames = ioElement.Element(XName.Get("DO")).Attribute(XName.Get("names")).Value.Split(',');
            string[] diNames = ioElement.Element(XName.Get("DI")).Attribute(XName.Get("names")).Value.Split(',');
            string[] aoNames = ioElement.Element(XName.Get("AO")).Attribute(XName.Get("names")).Value.Split(',');
            string[] aiNames = ioElement.Element(XName.Get("AI")).Attribute(XName.Get("names")).Value.Split(',');

            var io = new RobotIO() { DO = doNames, DI = diNames, AO = aoNames, AI = aiNames };

            switch (manufacturer)
            {
                case (Manufacturers.ABB):
                    return new RobotABB(modelName, basePlane, baseMesh, joints, io);
                case (Manufacturers.KUKA):
                    return new RobotKUKA(modelName, basePlane, baseMesh, joints, io);
                case (Manufacturers.UR):
                    return new RobotUR(modelName, basePlane, baseMesh, joints, io);
                default:
                    return null;
            }
        }

        public static void WriteMeshes()
        {
            Rhino.FileIO.File3dm robotsGeometry = Rhino.FileIO.File3dm.Read($@"{ResourcesFolder}\robotsGeometry.3dm");
            var robotMeshes = new RobotMeshes();

            foreach (var layer in robotsGeometry.Layers)
            {
                if (layer.Name == "Default" || layer.ParentLayerId != Guid.Empty) continue;
                robotMeshes.Names.Add(layer.Name);
                var meshes = new Mesh[7];
                meshes[0] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == layer.LayerIndex).Geometry as Mesh;

                for (int i = 0; i < 6; i++)
                {
                    meshes[i + 1] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == layer.LayerIndex + 1 + i).Geometry as Mesh;
                }
                robotMeshes.Meshes.Add(meshes);
            }

            using (var stream = new MemoryStream())
            {
                var formatter = new BinaryFormatter();
                formatter.Serialize(stream, robotMeshes);
                File.WriteAllBytes($@"{ResourcesFolder}\Meshes.rob", stream.ToArray());
            }
        }


        public virtual KinematicSolution Kinematics(Target target, bool calculateMeshes = false) => new SphericalWristKinematics(target, this, calculateMeshes);

        public override string ToString() => $"Robot ({Model})";

        internal abstract List<string> Code(Program program);
        protected abstract double[] GetStartPose();
        public abstract double DegreeToRadian(double degree, int i);
        public abstract double RadianToDegree(double radian, int i);

        internal class Joint
        {
            public double A { get; set; }
            public double D { get; set; }
            public Interval Range { get; set; }
            public double MaxSpeed { get; set; }
            public Plane Plane { get; set; }
            public Mesh Mesh { get; set; }
        }

        public class RobotIO
        {
            public string[] DO { get; internal set; }
            public string[] DI { get; internal set; }
            public string[] AO { get; internal set; }
            public string[] AI { get; internal set; }
        }

        [Serializable]
        class RobotMeshes
        {
            internal List<string> Names { get; set; } = new List<string>();
            internal List<Mesh[]> Meshes { get; set; } = new List<Mesh[]>();
        }
    }
}