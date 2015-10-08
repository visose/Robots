using System;
using System.Text;

using Rhino.Geometry;
using static Robots.Util;

namespace Robots
{
    [Serializable]
    class RobotABB : Robot
    {
        public RobotABB(string model, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturer.ABB, "MOD", basePlane, baseMesh, joints) { }

        public override StringBuilder Code(Program program) => null;
    }
}