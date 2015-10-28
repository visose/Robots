using System;
using System.Text;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots
{
    [Serializable]
    class RobotABB : Robot
    {
        internal RobotABB(string model, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io) : base(model, Manufacturers.ABB, "MOD", basePlane, baseMesh, joints, io) { }

        internal override List<string> Code(Program program) => new RapidPostProcessor(this, program).Code;

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree * PI / 180;
            if (i == 1) radian = -radian+PI*0.5;
            if (i == 2) radian *= -1;
            if (i == 4) radian *= -1;
            return radian;
        }

        public override double RadianToDegree(double radian, int i)
        {
            if (i == 1) { radian -= PI * 0.5; radian = -radian; }
            if (i == 2) radian *= -1;
            if (i == 4) radian *= -1;
            double degree = radian * 180 / PI;
            return degree;
        }

        class RapidPostProcessor
        {
            RobotABB robot;
            Program program;
            internal List<string> Code { get; }

            internal RapidPostProcessor(RobotABB robot, Program program)
            {
                this.robot = robot;
                this.program = program;
                this.Code = PostProcessor();
            }

            List<string> PostProcessor()
            {
                var code = new List<string>();
                code.Add("MODULE MainModule");
                code.Add("ConfJ \\Off;");
                code.Add("ConfL \\Off;");
                code.Add("VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];");
                code.Add("VAR confdata conf := [0,-2,1,0];");

                var tools = program.Targets.Select(x => x.Tool).Distinct();
                var speeds = program.Targets.Select(x => x.Speed).Distinct();
                var zones = program.Targets.Select(x => x.Zone).Distinct();

                foreach (var tool in tools)
                    code.Add(Tool(tool));

                foreach (var speed in speeds)
                {
                    int i = 0;
                    code.Add(Speed(speed,i++));
                }

                foreach (var zone in zones)
                {
                    int i = 0;
                    if (zone.IsFlyBy) code.Add(Zone(zone, i++));
                }

                code.Add("PROC Main()");

                string initCommands = program.InitCommands.Code(program.Robot, new Target(Plane.Unset));
                if (initCommands.Length > 0)
                    code.Add(initCommands);

                Plane originPlane = new Plane(Point3d.Origin, -Vector3d.YAxis, Vector3d.XAxis);

                foreach (Target target in program.Targets)
                {
                    Plane plane = Plane.Unset;
                    Quaternion quaternion = Quaternion.Zero;
                    string moveText = null;

                    if (target.Motion != Target.Motions.JointRotations)
                    {
                        plane = target.Plane;
                        plane.Transform(Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY));
                        quaternion = Quaternion.Rotation(Plane.WorldXY, plane);
                    }

                    switch (target.Motion)
                    {
                        case Target.Motions.JointRotations:
                            {
                                double[] joints = target.IsCartesian ? robot.Kinematics(target, false).JointRotations : target.JointRotations;
                                joints = joints.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                                 moveText = $"MoveAbsJ [[{joints[0]:0.000}, {joints[1]:0.000}, {joints[2]:0.000}, {joints[3]:0.000}, {joints[4]:0.000}, {joints[5]:0.000}],[0,9E9,9E9,9E9,9E9,9E9]],{target.Speed.Name},{target.Zone.Name},{target.Tool.Name};";                                
                                break;
                            }

                        case Target.Motions.JointCartesian:
                            {
                                string pos = $"[{plane.OriginX:0.00},{plane.OriginY:0.00},{plane.OriginZ:0.00}]";
                                string orient = $"[{quaternion.A:0.0000},{quaternion.B:0.0000},{quaternion.C:0.0000},{quaternion.D:0.0000}]";
                                string robtarget = $"[{pos},{orient},conf,extj]";
                                string zone = target.Zone.IsFlyBy ? target.Zone.Name : "fine";
                                moveText = $"MoveJ {robtarget}, {target.Speed.Name}, {zone}, {target.Tool.Name};";
                                break;
                            }

                        case Target.Motions.Linear:
                            {
                                string pos = $"[{plane.OriginX:0.00},{plane.OriginY:0.00},{plane.OriginZ:0.00}]";
                                string orient = $"[{quaternion.A:0.0000},{quaternion.B:0.0000},{quaternion.C:0.0000},{quaternion.D:0.0000}]";
                                string robtarget = $"[{pos},{orient},conf,extj]";
                                string zone = target.Zone.IsFlyBy ? target.Zone.Name : "fine";
                                moveText = $"MoveL {robtarget}, {target.Speed.Name}, {zone}, {target.Tool.Name};";
                                break;
                            }
                    }

                    code.Add(moveText);

                    string commands = target.Commands.Code(program.Robot, target);
                    if (commands.Length > 0)
                        code.Add(commands);
                }

                code.Add("ENDPROC");
                code.Add("ENDMODULE");
                return code;
            }

            string Tool(Tool tool)
            {
                string pos = $"[{tool.Tcp.OriginX:0.00},{tool.Tcp.OriginY:0.00},{tool.Tcp.OriginZ:0.00}]";
                Quaternion quaternion = Quaternion.Rotation(Plane.WorldXY, tool.Tcp);
                string orient = $"[{quaternion.A:0.000},{quaternion.B:0.000},{quaternion.C:0.000},{quaternion.D:0.000}]";
                Point3d centroid = tool.Tcp.Origin / 2;
                string loaddata = $"[{tool.Weight:0.000},[{centroid.X:0.00},{centroid.Y:0.00},{centroid.Z:0.00}],[1,0,0,0],0,0,0]";
                return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
            }

            string Speed(Speed speed, int i)
            {
                if (speed.Name == null) speed.Name = $"speed{i:000}";
                return $"VAR speeddata {speed.Name}:=[{speed.TranslationSpeed:0.00},{speed.RotationSpeed:0.00},200,15];";
            }

            string Zone(Zone zone, int i)
            {
                if (zone.Name == null) zone.Name = $"zone{i:000}";
                string stop = zone.IsFlyBy ? "FALSE" : "TRUE";
                double angle = 0.1;
                return $"VAR zonedata {zone.Name}:=[{stop},{zone.Distance:0.00},{zone.Rotation:0.00},{zone.Distance:0.00},{angle:0.00},{zone.Rotation:0.00},{angle:0.00}];";
            }
        }

    }
}