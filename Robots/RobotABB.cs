using System;
using System.Text;
using System.Linq;
using System.Collections.Generic;

using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots
{
    class RobotABB : Robot
    {
        internal RobotABB(string model, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io) : base(model, basePlane, baseMesh, joints, io)
        {
            this.Manufacturer = Manufacturers.ABB;
            this.Extension = "MOD";
        }

        internal override List<string> Code(Program program) => new RapidPostProcessor(this, program).Code;

        protected override double[] GetStartPose() => new double[] { 0, PI / 2, 0, 0, 0, 0 };

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree * PI / 180;
            if (i == 1) radian = -radian + PI * 0.5;
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
                code.Add("VAR extjoint extj := [9E9,9E9,9E9,9E9,9E9,9E9];");
                // code.Add("VAR confdata conf := [0,-2,1,0];");
                code.Add("VAR confdata conf := [0,0,0,0];");

                var tools = program.Targets.Select(x => x.Tool).Distinct().ToList();
                var speeds = program.Targets.Select(x => x.Speed).Distinct().ToList();
                var zones = program.Targets.Select(x => x.Zone).Distinct().ToList();

                foreach (var tool in tools)
                    code.Add(Tool(tool));

                for (int i = 0; i < speeds.Count; i++)
                    code.Add(Speed(speeds[i], i));

                for (int i = 0; i < zones.Count; i++)
                    if (zones[i].IsFlyBy) code.Add(Zone(zones[i], i));

                code.Add("PROC Main()");
                code.Add("ConfJ \\Off;");
                code.Add("ConfL \\Off;");

                string initCommands = program.InitCommands.Code(program.Robot, new Target(Plane.Unset));
                if (initCommands.Length > 0)
                    code.Add(initCommands);

                Plane originPlane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);

                foreach (Target target in program.Targets)
                {
                    Plane plane = Plane.Unset;
                    Quaternion quaternion = Quaternion.Zero;
                    string moveText = null;

                    if (target.Motion != Target.Motions.JointRotations)
                    {
                        plane = target.Plane;
                        plane.Transform(Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY));
                        quaternion = Quaternion.Rotation(originPlane, plane);
                    }

                    switch (target.Motion)
                    {
                        case Target.Motions.JointRotations:
                            {
                                double[] joints = target.JointRotations;
                                joints = joints.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                                string zone = target.Zone.IsFlyBy ? target.Zone.Name : "fine";
                                moveText = $"MoveAbsJ [[{joints[0]:0.000}, {joints[1]:0.000}, {joints[2]:0.000}, {joints[3]:0.000}, {joints[4]:0.000}, {joints[5]:0.000}],extj],{target.Speed.Name},{zone},{target.Tool.Name};";
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
                Quaternion quaternion = Quaternion.Rotation(Plane.WorldXY, tool.Tcp);
                double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

                Point3d centroid = tool.Tcp.Origin / 2;
                if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                    centroid = new Point3d(0, 0, 0.001);

                string pos = $"[{tool.Tcp.OriginX:0.000},{tool.Tcp.OriginY:0.000},{tool.Tcp.OriginZ:0.000}]";
                string orient = $"[{quaternion.A:0.0000},{quaternion.B:0.0000},{quaternion.C:0.0000},{quaternion.D:0.0000}]";
                string loaddata = $"[{weight:0.000},[{centroid.X:0.000},{centroid.Y:0.000},{centroid.Z:0.000}],[1,0,0,0],0,0,0]";
                return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
            }

            string Speed(Speed speed, int i)
            {
                if (speed.Name == null) speed.Name = $"speed{i:000}";
                double rotaion = speed.RotationSpeed * (180 / PI);
                return $"VAR speeddata {speed.Name}:=[{speed.TranslationSpeed:0.00},{rotaion:0.00},200,15];";
            }

            string Zone(Zone zone, int i)
            {
                if (zone.Name == null) zone.Name = $"zone{i:000}";
                double angle = 0.1;
                return $"VAR zonedata {zone.Name}:=[FALSE,{zone.Distance:0.00},{zone.Rotation:0.00},{zone.Distance:0.00},{angle:0.00},{zone.Rotation:0.00},{angle:0.00}];";
            }
        }

    }
}