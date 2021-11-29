using static System.Math;

namespace Robots
{
    public class Speed : TargetAttribute
    {
        /// <summary>
        /// TCP translation speed in mm/s
        /// </summary>
        public double TranslationSpeed { get; set; }
        /// <summary>
        /// TCP rotation speed in rad/s
        /// </summary>
        public double RotationSpeed { get; set; }
        /// <summary>
        /// Translation speed in mm/s
        /// </summary>
        public double TranslationExternal { get; set; }
        /// <summary>
        /// Rotation speed in rad/s
        /// </summary>
        public double RotationExternal { get; set; }
        /// <summary>
        /// Translation acceleration in mm/s² (used in UR)
        /// </summary>
        public double TranslationAccel { get; set; } = 1000;
        /// <summary>
        /// Axis/join acceleration in rads/s² (used in UR)
        /// </summary>
        public double AxisAccel { get; set; } = PI;

        /// <summary>
        /// Time in seconds it takes to reach the target. Optional parameter (used in UR)
        /// </summary>
        public double Time { get; set; } = 0;

        public static Speed Default { get; }

        static Speed()
        {
            Default = new Speed(name: "DefaultSpeed");
        }

        public Speed(double translation = 100, double rotationSpeed = PI, double translationExternal = 5000, double rotationExternal = PI * 6, string? name = null)
        {
            Name = name;
            TranslationSpeed = translation;
            RotationSpeed = rotationSpeed;
            TranslationExternal = translationExternal;
            RotationExternal = rotationExternal;
        }

        public override string ToString() => (Name != null) ? $"Speed ({Name})" : $"Speed ({TranslationSpeed:0.0} mm/s)";
    }
}