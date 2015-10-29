using System;
using System.Text;
using System.Linq;
using System.Collections.Generic;
using static System.Math;

using Rhino.Geometry;
using static Robots.Util;
using static Rhino.RhinoMath;


namespace Robots
{
    public class RobotKUKA : Robot
    {
        internal RobotKUKA(string model, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io) : base(model, Manufacturers.KUKA, "SRC", basePlane, baseMesh, joints, io) { }

        internal override List<string> Code(Program program) => new KRLPostProcessor(this, program).Code;

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree * PI / 180;
            if (i == 2) radian -= 0.5 * PI;
            if (i == 5) radian += PI;
            radian = -radian;
            return radian;
        }

        public override double RadianToDegree(double radian, int i)
        {
            double degree = -radian;
            if (i == 2) degree += 0.5 * PI;
            if (i == 5) degree -= PI;
            degree = degree * 180 / PI;
            return degree;
        }


        /// <summary>
        /// Lifted from unknown source.
        /// </summary>
        /// <param name="basePlane"></param>
        /// <param name="targetPlane"></param>
        /// <returns></returns>
        public static double[] EulerAngles(Plane basePlane, Plane targetPlane)
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

        class KRLPostProcessor
        {
            RobotKUKA robot;
            Program program;
            internal List<string> Code { get; }

            internal KRLPostProcessor(RobotKUKA robot, Program program)
            {
                this.robot = robot;
                this.program = program;
                this.Code = PostProcessor();
            }

            List<string> PostProcessor()
            {
                var code = new List<string>();

                code.Add($@"&ACCESS RVP
&REL 1
&PARAM TEMPLATE = C:\KRC\Roboter\Template\vorgabe
&PARAM EDITMASK = *
DEF {program.Name}()
;Init Code
BAS (#INITMOV,0)
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
$APO.CPTP=100
");

                string initCommands = program.InitCommands.Code(program.Robot, new Target(Plane.Unset));
                if (initCommands.Length > 0)
                    code.Add(initCommands);

                Tool currentTool = null;
                Speed currentSpeed = null;
                Zone currentZone = null;
                Plane KukaBasePlane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);

                foreach (Target target in program.Targets)
                {
                    if (currentTool == null || target.Tool != currentTool)
                    {
                        code.Add(Tool(target.Tool));
                        currentTool = target.Tool;
                    }

                    if (target.Zone.IsFlyBy && (currentZone == null || target.Zone != currentZone))
                    {
                        code.Add($"$APO.CDIS={target.Zone.Distance:0.000}");
                        currentZone = target.Zone;
                    }

                    if (currentSpeed == null || target.Speed != currentSpeed)
                    {
                        code.Add($"$VEL={{CP {target.Speed.TranslationSpeed / 1000:0.00000},ORI1 {target.Speed.RotationSpeed:0.000},ORI2 {target.Speed.RotationSpeed:0.000} }}");
                        currentSpeed = target.Speed;
                    }

                    Plane plane = Plane.Unset;
                    double[] euler = null;
                    string moveText = null;

                    if (target.Motion != Target.Motions.JointRotations)
                    {
                        plane = target.Plane;
                        plane.Transform(Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY));
                        euler = EulerAngles(KukaBasePlane, plane);
                    }

                    switch (target.Motion)
                    {
                        case Target.Motions.JointRotations:
                            {
                                double[] joints = target.IsCartesian ? robot.Kinematics(target, false).JointRotations : target.JointRotations;
                                joints = joints.Select((x, i) => robot.RadianToDegree(x,i)).ToArray();
                                moveText = $"PTP {{A1 {joints[0]:0.000},A2 {joints[1]:0.000},A3 {joints[2]:0.000},A4 {joints[3]:0.000},A5 {joints[4]:0.000},A6 {joints[5]:0.000},E1 0,E2 0,E3 0,E4 0,E5 0,E6 0}}";
                                if (target.Zone.IsFlyBy) moveText += " C_PTP";                                
                                break;
                            }

                        case Target.Motions.JointCartesian:
                            {
                                moveText = $"PTP {{X {plane.OriginX:0.00},Y {plane.OriginY:0.00},Z {plane.OriginZ:0.00},A {euler[0]:0.000},B {euler[1]:0.000},C {euler[2]:0.000} }}";
                                if (target.Zone.IsFlyBy) moveText += " C_PTP";
                                break;
                            }

                        case Target.Motions.Linear:
                            {
                                moveText = $"LIN {{X {plane.OriginX:0.00},Y {plane.OriginY:0.00},Z {plane.OriginZ:0.00},A {euler[0]:0.000},B {euler[1]:0.000},C {euler[2]:0.000} }}";
                                if (target.Zone.IsFlyBy) moveText += " C_DIS";
                                break;
                            }
                    }

                    code.Add(moveText);

                    string commands = target.Commands.Code(program.Robot, target);
                    if (commands.Length > 0)
                        code.Add(commands);
                }

                code.Add("END");
                return code;
            }

            string Tool(Tool tool)
            {
                var basePlane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);
                double[] eulerAngles = EulerAngles(basePlane, tool.Tcp);
                string toolTxt =$"$TOOL={{X {tool.Tcp.OriginX:0.000},Y {tool.Tcp.OriginY:0.000},Z {tool.Tcp.OriginZ:0.000},A {eulerAngles[0]:0.000},B {eulerAngles[1]:0.000},C {eulerAngles[2]:0.000} }} ;{tool.Name}";
                string load = $"$LOAD.M={tool.Weight}";
                Point3d centroid = tool.Tcp.Origin / 2;
                string centroidTxt = $"$LOAD.CM={{X {centroid.X:0.000},Y {centroid.Y:0.000},Z {centroid.Z:0.000},A 0,B 0,C 0}}";
                return $"{toolTxt}\r\n{load}\r\n{centroidTxt}";
            }
        }
    }
}