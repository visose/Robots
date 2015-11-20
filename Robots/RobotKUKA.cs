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
        internal RobotKUKA(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io) : base(model, payload, basePlane, baseMesh, joints, io)
        {
            this.Manufacturer = Manufacturers.KUKA;
            this.Extension = "SRC";
        }

        internal override List<string> Code(Program program) => new KRLPostProcessor(this, program).Code;

        protected override double[] GetStartPose() => new double[] { 0, PI / 2, 0, 0, 0, -PI };

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree * PI / 180;
            if (i == 2) radian -= 0.5 * PI;
            // if (i == 5) radian += PI;
            radian = -radian;
            return radian;
        }

        public override double RadianToDegree(double radian, int i)
        {
            double degree = -radian;
            if (i == 2) degree += 0.5 * PI;
            // if (i == 5) degree -= PI;
            degree = degree * 180 / PI;
            return degree;
        }


        /// <summary>
        /// Lifted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToEuler
        /// </summary>
        /// <param name="basePlane"></param>
        /// <param name="targetPlane"></param>
        /// <returns></returns>
        public static double[] EulerAngles(Plane basePlane, Plane targetPlane)
        {
            Transform matrix = Transform.PlaneToPlane(basePlane, targetPlane);
            double heading, attitude, bank;

            if (matrix.M10 > Tol)
            { // singularity at north pole
                heading = Atan2(matrix.M02, matrix.M22);
                attitude = PI / 2;
                bank = 0;
            }
            else if (matrix.M10 < -Tol)
            { // singularity at south pole
                heading = Atan2(matrix.M02, matrix.M22);
                attitude = -PI / 2;
                bank = 0;
            }
            else
            {
                heading = Atan2(-matrix.M20, matrix.M00);
                bank = Atan2(-matrix.M12, matrix.M11);
                attitude = Asin(matrix.M10);
            }

            double[] values = new double[] { heading, attitude, bank };

            for (int i = 0; i < 3; i++)
            {
                values[i] *= 180.0 / PI;
                if (Abs(values[i] - -180.0) < Tol) values[i] = 180.0;
            }

            return values;
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
INT I
BAS (#INITMOV,0)
FOR I=1 TO 6
$VEL_AXIS[I] = 100 ;all axis velocities to 100%
ENDFOR
$ADVANCE=5
$APO.CPTP=100
");

                string initCommands = program.InitCommands.Code(program.Robot, new Target(Plane.Unset));
                if (initCommands.Length > 0)
                    code.Add(initCommands);

                Tool currentTool = null;
                Speed currentSpeed = null;
                Zone currentZone = null;
                Plane KukaBasePlane = new Plane(Point3d.Origin, Vector3d.XAxis, Vector3d.YAxis);

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
                        double rotation = target.Speed.RotationSpeed * 180 / PI;
                        if (target.Motion != Target.Motions.JointCartesian && target.Motion != Target.Motions.JointRotations)
                        {
                            code.Add($"$VEL={{CP {target.Speed.TranslationSpeed / 1000:0.00000},ORI1 {rotation:0.000},ORI2 {rotation:0.000} }}");
                            currentSpeed = target.Speed;
                        }
                    }

                    if (target.Motion == Target.Motions.JointCartesian || target.Motion == Target.Motions.JointRotations)
                    {
                        double percentage = (target.Speed.AxisSpeed > 0) ? target.Speed.AxisSpeed : (target.Time > 0) ? target.MinTime / target.Time : 0.1;
                        percentage *= 100;
                        code.Add($"BAS(#VEL_PTP, {percentage:0})");
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
                                double[] joints = target.JointRotations;
                                joints = joints.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                                moveText = $"PTP {{A1 {joints[0]:0.000},A2 {joints[1]:0.000},A3 {joints[2]:0.000},A4 {joints[3]:0.000},A5 {joints[4]:0.000},A6 {joints[5]:0.000},E1 0,E2 0,E3 0,E4 0,E5 0,E6 0}}";
                                if (target.Zone.IsFlyBy) moveText += " C_PTP";
                                break;
                            }

                        case Target.Motions.JointCartesian:
                            {
                                string bits = string.Empty;
                                if (target.ChangesConfiguration)
                                {
                                    int turnNum = 0;
                                    double[] degrees = target.jointRotations.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                                    for (int i = 0; i < 6; i++) if (degrees[i] < 0) turnNum += (int)Pow(2, i);

                                    var configuration = target.Configuration;
                                    bool shoulder = configuration.HasFlag(Target.RobotConfigurations.Shoulder);
                                    bool elbow = configuration.HasFlag(Target.RobotConfigurations.Elbow);
                                    elbow = !elbow;
                                    bool wrist = configuration.HasFlag(Target.RobotConfigurations.Wrist);

                                    int configNum = 0;
                                    if (shoulder) configNum += 1;
                                    if (elbow) configNum += 2;
                                    if (wrist) configNum += 4;

                                    string status = Convert.ToString(configNum, 2);
                                    string turn = Convert.ToString(turnNum, 2);
                                    bits = $@", S'B{status:000}',T'B{turn:000000}'";
                                }

                                moveText = $"PTP {{X {plane.OriginX:0.00},Y {plane.OriginY:0.00},Z {plane.OriginZ:0.00},A {euler[0]:0.000},B {euler[1]:0.000},C {euler[2]:0.000}{bits} }}";
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
                Plane tcp = tool.Tcp;
                tcp = new Plane(tcp.Origin, tcp.YAxis, -tcp.ZAxis);
                tcp.Transform(Transform.PlaneToPlane(Plane.WorldXY, basePlane));

                double[] eulerAngles = EulerAngles(Plane.WorldXY, tcp);
                string toolTxt = $"$TOOL={{X {tcp.OriginX:0.000},Y {tcp.OriginY:0.000},Z {tcp.OriginZ:0.000},A {eulerAngles[0]:0.000},B {eulerAngles[1]:0.000},C {eulerAngles[2]:0.000} }} ;{tool.Name}";
                string load = $"$LOAD.M={tool.Weight}";
                Point3d centroid = tcp.Origin / 2;
                string centroidTxt = $"$LOAD.CM={{X {centroid.X:0.000},Y {centroid.Y:0.000},Z {centroid.Z:0.000},A 0,B 0,C 0}}";
                return $"{toolTxt}\r\n{load}\r\n{centroidTxt}";
            }
        }
    }
}