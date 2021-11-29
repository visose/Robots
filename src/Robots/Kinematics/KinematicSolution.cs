using Rhino.Geometry;
using System.Collections.Generic;

namespace Robots
{
    public abstract class KinematicSolution
    {
        public double[] Joints { get; protected set; } = new double[0];
        public Plane[] Planes { get; internal set; } = new Plane[0];
        public List<string> Errors { get; internal set; } = new List<string>();
        public RobotConfigurations Configuration { get; internal set; }
    }
}