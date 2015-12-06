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
        public double Payload { get; }
        internal RobotIO IO { get; }
        public Mesh DisplayMesh { get; }
        protected Plane basePlane;
        readonly Mesh baseMesh;
        internal Joint[] Joints { get; }

      //  public Plane[] GetPlanes() => Joints.Select(x => x.Plane).ToArray();
        

        internal Robot(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io)
        {
            this.model = model;
            this.Payload = payload;
            this.basePlane = basePlane;
            this.baseMesh = baseMesh;
            this.Joints = joints;
            this.IO = io;

            for (int i = 0; i < 6; i++)
            {
                joints[i].Range = new Interval(DegreeToRadian(joints[i].Range.T0, i), DegreeToRadian(joints[i].Range.T1, i));

                if (joints[i].MaxSpeed == 0)
                    joints[i].MaxSpeed = double.MaxValue;
                else
                    joints[i].MaxSpeed = joints[i].MaxSpeed.ToRadians();
            }

            var kinematics = Kinematics(new JointTarget(GetStartPose()));
            for (int i = 0; i < 6; i++)
            {
                Plane plane = kinematics.Planes[i + 1];
                plane.Transform(Transform.PlaneToPlane(basePlane, Plane.WorldXY));
                joints[i].Plane = plane;
            }

            this.DisplayMesh = CreateDisplayMesh();
        }

        internal Mesh CreateDisplayMesh()
        {
            var mesh = new Mesh();
            var transform = Transform.PlaneToPlane(Plane.WorldXY, basePlane);
            {
                var dupMesh = baseMesh.DuplicateMesh();
                dupMesh.Transform(transform);
                mesh.Append(dupMesh);
            }

            for (int i = 0; i < 6; i++)
            {
                var dupMesh = Joints[i].Mesh.DuplicateMesh();
                dupMesh.Transform(transform);
                mesh.Append(dupMesh);
            }

            return mesh;
        }

        public static List<string> ListRobots(bool fromFile = false)
        {
            var names = new List<string>();
            XElement robotsData = null;

            if (fromFile)
            {
                string dataFile = $@"{AssemblyDirectory}\robotsData.xml";
                if (!File.Exists(dataFile)) throw new FileNotFoundException($@"Can't find the file: {dataFile}");
                robotsData = XElement.Load(dataFile);
            }
            else
                robotsData = XElement.Parse(Properties.Resources.robotsData);

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
            string dataFile = $@"{AssemblyDirectory}\robotsData.xml";
            string geometryFile = $@"{AssemblyDirectory}\robotsGeometry.3dm";

            if (!File.Exists(dataFile)) throw new FileNotFoundException($@"Can't find the file: {dataFile}");
            if (!File.Exists(geometryFile)) throw new FileNotFoundException($@"Can't find the file: {geometryFile}");

            XElement robotsData = XElement.Load(dataFile);
            XElement robotElement = null;

            try
            {
                robotElement = robotsData.Elements().First(x => model == $"{x.Attribute(XName.Get("manufacturer")).Value}.{x.Attribute(XName.Get("model")).Value}");
            }
            catch (InvalidOperationException)
            {

                throw new InvalidOperationException($" Robot \"{model}\" is not in the data file");
            }

            Rhino.FileIO.File3dm robotsGeometry = Rhino.FileIO.File3dm.Read(geometryFile);
            Rhino.DocObjects.Layer robotLayer = null;

            try
            {
                robotLayer = robotsGeometry.Layers.First(x => x.Name == $"{model}");
            }
            catch (InvalidOperationException)
            {

                throw new InvalidOperationException($" Robot \"{model}\" is not in the geometry file");
            }

            var meshes = new Mesh[7];
            meshes[0] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == robotLayer.LayerIndex).Geometry as Mesh;

            for (int i = 0; i < 6; i++)
            {
                var jointLayer = robotsGeometry.Layers.First(x => (x.Name == $"{i + 1}") && (x.ParentLayerId == robotLayer.Id));
                meshes[i + 1] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == jointLayer.LayerIndex).Geometry as Mesh;
            }

            return Load(robotElement, meshes, basePlane);
        }

        private static Robot Load(XElement robotElement, Mesh[] meshes, Plane basePlane)
        {
            var modelName = robotElement.Attribute(XName.Get("model")).Value;
            var manufacturer = (Manufacturers)Enum.Parse(typeof(Manufacturers), robotElement.Attribute(XName.Get("manufacturer")).Value);
            var baseMesh = meshes[0].DuplicateMesh();
            double payload = Convert.ToDouble(robotElement.Attribute(XName.Get("payload")).Value);

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

                joints[i] = new Joint() { Index = i,  A = a, D = d, Range = range, MaxSpeed = maxSpeed, Mesh = mesh };
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
                    return new RobotABB(modelName, payload, basePlane, baseMesh, joints, io);
                case (Manufacturers.KUKA):
                    return new RobotKUKA(modelName, payload, basePlane, baseMesh, joints, io);
                case (Manufacturers.UR):
                    return new RobotUR(modelName, payload, basePlane, baseMesh, joints, io);
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
                    var jointLayer = robotsGeometry.Layers.First(x => (x.Name == $"{i + 1}") && (x.ParentLayerId == layer.Id));
                    meshes[i + 1] = robotsGeometry.Objects.First(x => x.Attributes.LayerIndex == jointLayer.LayerIndex).Geometry as Mesh;
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
            internal int Index { get; set; }
            internal double A { get; set; }
            internal double D { get; set; }
            internal Interval Range { get; set; }
            internal double MaxSpeed { get; set; }
            internal Plane Plane { get; set; }
            internal Mesh Mesh { get; set; }
        }

        internal class RobotIO
        {
            internal string[] DO { get; set; }
            internal string[] DI { get; set; }
            internal string[] AO { get; set; }
            internal string[] AI { get; set; }
        }

        [Serializable]
        class RobotMeshes
        {
            internal List<string> Names { get; set; } = new List<string>();
            internal List<Mesh[]> Meshes { get; set; } = new List<Mesh[]>();
        }
    }
}