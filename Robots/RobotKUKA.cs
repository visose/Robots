using System;
using System.Text;
using static System.Math;

using Rhino.Geometry;
using static Robots.Util;


namespace Robots
{
    [Serializable]
    public class RobotKUKA : Robot
    {
        public RobotKUKA(string model, Plane basePlane, Mesh baseMesh, Joint[] joints) : base(model, Manufacturer.KUKA, "SRC", basePlane, baseMesh, joints) { }

        public override StringBuilder Code(Program program)
        {
            var code = new StringBuilder();

            code.AppendLine($@"&ACCESS RVP
&REL 1
&COMMENT {program.Name} PROGRAM
&PARAM TEMPLATE = C:\KRC\Roboter\Template\vorgabe
&PARAM EDITMASK = *
DEF {program.Name}()
;Init Code
BAS (#INITMOV,0)
BAS (#VEL_PTP,15)
$VEL_AXIS[1]=100
$VEL_AXIS[2]=100
$VEL_AXIS[3]=100
$VEL_AXIS[4]=100
$VEL_AXIS[5]=100
$VEL_AXIS[6]=100
$ACC_AXIS[1]=100
$ACC_AXIS[2]=100
$ACC_AXIS[3]=100
$ACC_AXIS[4]=100
$ACC_AXIS[5]=100
$ACC_AXIS[6]=100
$ADVANCE=5
$APO.CPTP=15
$APO.CVEL=100

$ANOUT[1]=0.274+$VEL_ACT*8.0
");

            Tool currentTool = null;
            double currentSpeed = -1;
            double currentZone = -1;

            foreach (Target target in program.Targets)
            {
                if (currentTool == null || target.Tool != currentTool)
                {
                    code.AppendLine(Tool(target.Tool));
                    currentTool = target.Tool;
                }

                if (target.Zone.IsFlyBy && (currentZone == -1 || target.Zone.Distance != currentZone))
                {
                    code.AppendLine($"$APO.CDIS={target.Zone.Distance:0.000}");
                    currentZone = target.Zone.Distance;
                }

                if (currentSpeed == -1 || target.Speed.TranslationSpeed != currentSpeed)
                {
                    code.AppendLine($"$VEL={{CP {target.Speed.TranslationSpeed / 1000:0.000},ORI1 360,ORI2 360}}");
                    currentSpeed = target.Speed.TranslationSpeed;
                }

                string moveText = "";
                var basePlane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);

                switch (target.Motion)
                {
                    case Target.Motions.JointRotations:

                        double[] joints = target.IsCartesian? Kinematics(target, false).JointRotations : target.JointRotations;
                        moveText = $"PTP {{A1 {joints[0]:0.00},A2 {joints[1]:0.00},A3 {joints[2]:0.00},A4 {joints[3]:0.00},A5 {joints[4]:0.00},A6 {joints[5]:0.00},E1 0,E2 0,E3 0,E4 0,E5 0,E6 0}}";
                        if (target.Zone.IsFlyBy) moveText += " C_PTP";
                        break;

                    case Target.Motions.JointCartesian:
                    
                        double[] eulerJoint = EulerAngles(basePlane, target.Plane);
                        moveText = $"PTP {{X {target.Plane.OriginX:0.000},Y {target.Plane.OriginY:0.000},Z {target.Plane.OriginZ:0.000},A {eulerJoint[0]:0.000},B {eulerJoint[1]:0.000},C {eulerJoint[2]:0.000} }}";
                        if (target.Zone.IsFlyBy) moveText += " C_PTP";
                        break;

                    case Target.Motions.Linear:

                        double[] eulerLinear = EulerAngles(basePlane, target.Plane);
                        moveText = $"LIN {{X {target.Plane.OriginX:0.000},Y {target.Plane.OriginY:0.000},Z {target.Plane.OriginZ:0.000},A {eulerLinear[0]:0.000},B {eulerLinear[1]:0.000},C {eulerLinear[2]:0.000} }}";
                        if (target.Zone.IsFlyBy) moveText += " C_DIS";
                        break;
                }

                code.AppendLine(moveText);
                string commands = target.Commands.CodeKUKA();
                if (commands.Length>3)
                code.AppendLine(commands);
            }

            code.Append("END");

            return code;
        }

        string Tool(Tool tool)
        {
            var code = new StringBuilder();
            var basePlane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);
            double[] eulerAngles = EulerAngles(basePlane, tool.TCP);
            code.AppendLine($"$TOOL={{X {tool.TCP.OriginX:0.000},Y {tool.TCP.OriginY:0.000},Z {tool.TCP.OriginZ:0.000},A {eulerAngles[0]:0.000},B {eulerAngles[1]:0.000},C {eulerAngles[2]:0.000} }} ;{tool.Name}");
            code.AppendLine($"$LOAD.M={tool.Weight}");
            Point3d centroid = tool.TCP.Origin / 2;
            code.Append($"$LOAD.CM={{X {centroid.X:0.00},Y {centroid.Y:0.00},Z {centroid.Z:0.00},A 0,B 0,C 0}}");
            return code.ToString();
        }

        /// <summary>
        /// Lifted from unknown source.
        /// </summary>
        /// <param name="basePlane"></param>
        /// <param name="targetPlane"></param>
        /// <returns></returns>
        double[] EulerAngles(Plane basePlane, Plane targetPlane)
        {           
            Transform matrix = Transform.PlaneToPlane(basePlane, targetPlane);

            double r, p, w;
            if (Abs(matrix[2, 0] - 1) < 0.0000001 || Abs(matrix[2, 0] + 1) < 0.0000001)
            {
                p = Asin(-matrix[2, 0]);
                r = Asin(-matrix[0, 1] / matrix[2, 0]);
                w = 0;
            }
            else
            {
                r = Atan2(matrix[2, 1], matrix[2, 2]);
                p = Atan2(-matrix[2, 0], Sqrt(matrix[2, 1] * matrix[2, 1] + matrix[2, 2] * matrix[2, 2]));
                w = Atan2(matrix[1, 0], matrix[0, 0]);
            }
            r *= 180 / PI;
            p *= 180 / PI;
            w *= 180 / PI;
            return new double[] { w, p, r };
        }
    }
}
