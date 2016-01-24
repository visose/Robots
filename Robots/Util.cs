using System;
using static System.Math;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Reflection;
using System.IO;

namespace Robots
{
    static class Util
    {
        public const double DistanceTol = 0.001;
        public const double AngleTol = 0.001;
        public const double TimeTol = 0.0001;
        public const double UnitTol = 0.000001;
        internal const double SingularityTol = 0.0001;

        internal const string ResourcesFolder = @"C:\Users\Vicente\Documents\Trabajo\Bartlett\RobotsApp\Robots\Robots\Resources";

        internal static Transform ToTransform(this double[,] matrix)
        {
            var transform = new Transform();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    transform[i, j] = matrix[i, j];

            return transform;
        }

        internal static Plane ToPlane(this Transform transform)
        {
            Plane plane = Plane.WorldXY;
           // Plane plane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);
            plane.Transform(transform);
            return plane;
        }

        internal static string AssemblyDirectory
        {
            get
            {
                string codeBase = Assembly.GetExecutingAssembly().CodeBase;
                UriBuilder uri = new UriBuilder(codeBase);
                string path = Uri.UnescapeDataString(uri.Path);
                return Path.GetDirectoryName(path);
            }
        }

        internal static double ToRadians(this double value)
        {
            return value * (PI / 180);
        }

        internal static double ToDegrees(this double value)
        {
            return value * (180 / PI);
        }
    }
}