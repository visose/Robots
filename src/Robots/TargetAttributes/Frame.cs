using Rhino.Geometry;

namespace Robots
{
    public class Frame : TargetAttribute
    {
        public static Frame Default { get; }

        /// <summary>
        /// Reference frame of plane for a target
        /// </summary>
        public Plane Plane { get; internal set; }
        public int CoupledMechanism { get; }
        public int CoupledMechanicalGroup { get; }
        public bool IsCoupled { get { return (CoupledMechanicalGroup != -1); } }

        internal int CoupledPlaneIndex { get; set; }

        static Frame()
        {
            Default = new Frame(Plane.WorldXY, -1, -1, "DefaultFrame");
        }

        public Frame(Plane plane, int coupledMechanism = -1, int coupledMechanicalGroup = -1, string? name = null)
        {
            Name = name;
            Plane = plane;
            CoupledMechanism = coupledMechanism;
            CoupledMechanicalGroup = coupledMechanicalGroup;
        }

        public Frame ShallowClone() => (Frame)MemberwiseClone();

        public override string ToString()
        {
            if (Name != null)
                return $"Frame ({Name})";

            string origin = $"{Plane.OriginX:0.00},{Plane.OriginY:0.00},{Plane.OriginZ:0.00}";
            return $"Frame ({origin}" + (IsCoupled ? " Coupled" : "") + ")";
        }
    }
}