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
        internal RobotABB(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io) : base(model, payload, basePlane, baseMesh, joints, io)
        {
            this.Manufacturer = Manufacturers.ABB;
            this.Extension = "MOD";
        }

        internal override List<string> Code(Program program) => new RapidPostProcessor(this, program).Code;

        protected override double[] GetStartPose() => new double[] { 0, PI / 2, 0, 0, 0, 0 };

        public override double DegreeToRadian(double degree, int i)
        {
            double radian = degree.ToRadians();
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
            return radian.ToDegrees();
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
                code.Add("VAR confdata conf := [0,0,0,0];");

                foreach (var tool in program.Tools) code.Add(Tool(tool));
                foreach (var speed in program.Speeds) code.Add(Speed(speed));
                foreach (var zone in program.Zones) if (zone.IsFlyBy) code.Add(Zone(zone));

                code.Add("PROC Main()");
                //code.Add("ConfJ \\Off;");
                code.Add("ConfL \\Off;");

                string initCommands = program.InitCommands.Code(program.Robot, Target.Default);
                if (initCommands.Length > 0)
                    code.Add(initCommands);

                foreach (ProgramTarget target in program.Targets)
                {
                    string moveText = null;

                    if (target.IsJointTarget)
                    {
                        double[] joints = target.Joints;
                        joints = joints.Select((x, i) => robot.RadianToDegree(x, i)).ToArray();
                        string zone = target.Zone.IsFlyBy ? target.Zone.Name : "fine";
                        moveText = $"MoveAbsJ [[{joints[0]:0.000}, {joints[1]:0.000}, {joints[2]:0.000}, {joints[3]:0.000}, {joints[4]:0.000}, {joints[5]:0.000}],extj],{target.Speed.Name},{zone},{target.Tool.Name};";
                    }
                    else
                    {
                        var plane = target.Plane;
                        plane.Transform(Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY));
                        var quaternion = Quaternion.Rotation(Plane.WorldXY, plane);

                        switch (target.Motion)
                        {
                            case Target.Motions.Joint:
                                {
                                    string pos = $"[{plane.OriginX:0.00},{plane.OriginY:0.00},{plane.OriginZ:0.00}]";
                                    string orient = $"[{quaternion.A:0.0000},{quaternion.B:0.0000},{quaternion.C:0.0000},{quaternion.D:0.0000}]";

                                    int cf1 = (int)Floor(target.Joints[0] / (PI / 2));
                                    int cf4 = (int)Floor(target.Joints[3] / (PI / 2));
                                    int cf6 = (int)Floor(target.Joints[5] / (PI / 2));

                                    if (cf1 < 0) cf1--;
                                    if (cf4 < 0) cf4--;
                                    if (cf6 < 0) cf6--;

                                    Target.RobotConfigurations configuration = (Target.RobotConfigurations)target.Configuration;
                                    bool shoulder = configuration.HasFlag(Target.RobotConfigurations.Shoulder);
                                    bool elbow = configuration.HasFlag(Target.RobotConfigurations.Elbow);
                                    if (shoulder) elbow = !elbow;
                                    bool wrist = configuration.HasFlag(Target.RobotConfigurations.Wrist);

                                    int cfx = 0;
                                    if (wrist) cfx += 1;
                                    if (elbow) cfx += 2;
                                    if (shoulder) cfx += 4;

                                    string conf = $"[{cf1},{cf4},{cf6},{cfx}]";

                                    string robtarget = $"[{pos},{orient},{conf},extj]";
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
                Plane originPlane = new Plane(Point3d.Origin, -Vector3d.XAxis, -Vector3d.YAxis);
                Plane tcp = tool.Tcp;
                tcp.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));

                Quaternion quaternion = Quaternion.Rotation(Plane.WorldXY, tcp);
                double weight = (tool.Weight > 0.001) ? tool.Weight : 0.001;

                Point3d centroid = tcp.Origin / 2;
                if (centroid.DistanceTo(Point3d.Origin) < 0.001)
                    centroid = new Point3d(0, 0, 0.001);

                string pos = $"[{tcp.OriginX:0.000},{tcp.OriginY:0.000},{tcp.OriginZ:0.000}]";
                string orient = $"[{quaternion.A:0.0000},{quaternion.B:0.0000},{quaternion.C:0.0000},{quaternion.D:0.0000}]";
                string loaddata = $"[{weight:0.000},[{centroid.X:0.000},{centroid.Y:0.000},{centroid.Z:0.000}],[1,0,0,0],0,0,0]";
                return $"PERS tooldata {tool.Name}:=[TRUE,[{pos},{orient}],{loaddata}];";
            }

            string Speed(Speed speed)
            {
                double rotation = speed.RotationSpeed.ToDegrees();
                return $"VAR speeddata {speed.Name}:=[{speed.TranslationSpeed:0.00},{rotation:0.00},200,15];";
            }

            string Zone(Zone zone)
            {
                double angle = zone.Rotation.ToDegrees();
                return $"VAR zonedata {zone.Name}:=[FALSE,{zone.Distance:0.00},{zone.Distance:0.00},{zone.Distance:0.00},{angle:0.00},{zone.Distance:0.00},{angle:0.00}];";
            }
        }
    }
}