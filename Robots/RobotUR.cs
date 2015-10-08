using System;
using System.Text;

using Rhino.Geometry;
using static Robots.Util;

namespace Robots
{
    [Serializable]
    class RobotUR : Robot
    {
        public RobotUR(string model, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturer.UR, "URS", basePlane, baseMesh, joints) { }

        public override StringBuilder Code(Program program) => null;
    }
}
