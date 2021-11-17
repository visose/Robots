using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Net.Sockets;
using static System.Math;

namespace Robots
{
    public class RobotCellUR : RobotSystem
    {
        public RobotUR Robot { get; }
        //  public RemoteConnection Remote { get; } = new RemoteConnection();
        // public URRealTime URRealTime { get; set; }

        internal RobotCellUR(string name, RobotArm robot, IO io, Plane basePlane, Mesh environment) : base(name, Manufacturers.UR, io, basePlane, environment)
        {
            Remote = new RemoteUR();
            this.Robot = robot as RobotUR;
            this.DisplayMesh = new Mesh();
            DisplayMesh.Append(robot.DisplayMesh);
            this.DisplayMesh.Transform(this.BasePlane.ToTransform());
        }

        /// <summary>
        /// Code lifted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
        /// </summary>
        /// <param name="plane"></param>
        /// <param name="originPlane"></param>
        /// <returns></returns>
        public static double[] PlaneToAxisAngle(Plane plane, Plane originPlane)
        {
            Vector3d vector;
            Transform matrix = Transform.PlaneToPlane(originPlane, plane);

            double[][] m = new double[3][];
            m[0] = new double[] { matrix[0, 0], matrix[0, 1], matrix[0, 2] };
            m[1] = new double[] { matrix[1, 0], matrix[1, 1], matrix[1, 2] };
            m[2] = new double[] { matrix[2, 0], matrix[2, 1], matrix[2, 2] };

            double angle, x, y, z; // variables for result
            double epsilon = 0.01; // margin to allow for rounding errors
            double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees
                                   // optional check that input is pure rotation, 'isRotationMatrix' is defined at:
                                   // http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
                                   // assert isRotationMatrix(m) : "not valid rotation matrix";// for debugging
            if ((Abs(m[0][1] - m[1][0]) < epsilon)
              && (Abs(m[0][2] - m[2][0]) < epsilon)
            && (Abs(m[1][2] - m[2][1]) < epsilon))
            {
                // singularity found
                // first check for identity matrix which must have +1 for all terms
                //  in leading diagonal and zero in other terms
                if ((Abs(m[0][1] + m[1][0]) < epsilon2)
                  && (Abs(m[0][2] + m[2][0]) < epsilon2)
                  && (Abs(m[1][2] + m[2][1]) < epsilon2)
                && (Abs(m[0][0] + m[1][1] + m[2][2] - 3) < epsilon2))
                {
                    // this singularity is identity matrix so angle = 0
                    return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, 0, 0, 0 }; // zero angle, arbitrary axis
                }
                // otherwise this singularity is angle = 180
                angle = PI;
                double xx = (m[0][0] + 1) / 2;
                double yy = (m[1][1] + 1) / 2;
                double zz = (m[2][2] + 1) / 2;
                double xy = (m[0][1] + m[1][0]) / 4;
                double xz = (m[0][2] + m[2][0]) / 4;
                double yz = (m[1][2] + m[2][1]) / 4;
                if ((xx > yy) && (xx > zz))
                { // m[0][0] is the largest diagonal term
                    if (xx < epsilon)
                    {
                        x = 0;
                        y = 0.7071;
                        z = 0.7071;
                    }
                    else
                    {
                        x = Sqrt(xx);
                        y = xy / x;
                        z = xz / x;
                    }
                }
                else if (yy > zz)
                { // m[1][1] is the largest diagonal term
                    if (yy < epsilon)
                    {
                        x = 0.7071;
                        y = 0;
                        z = 0.7071;
                    }
                    else
                    {
                        y = Sqrt(yy);
                        x = xy / y;
                        z = yz / y;
                    }
                }
                else
                { // m[2][2] is the largest diagonal term so base result on this
                    if (zz < epsilon)
                    {
                        x = 0.7071;
                        y = 0.7071;
                        z = 0;
                    }
                    else
                    {
                        z = Sqrt(zz);
                        x = xz / z;
                        y = yz / z;
                    }
                }
                vector = new Vector3d(x, y, z);
                vector.Unitize();
                vector *= angle;
                return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, vector.X, vector.Y, vector.Z }; // return 180 deg rotation
            }
            // as we have reached here there are no singularities so we can handle normally
            double s = Sqrt((m[2][1] - m[1][2]) * (m[2][1] - m[1][2])
              + (m[0][2] - m[2][0]) * (m[0][2] - m[2][0])
              + (m[1][0] - m[0][1]) * (m[1][0] - m[0][1])); // used to normalise
            if (Abs(s) < 0.001) s = 1;
            // prevent divide by zero, should not happen if matrix is orthogonal and should be
            // caught by singularity test above, but I've left it in just in case
            angle = Acos((m[0][0] + m[1][1] + m[2][2] - 1) / 2);
            x = (m[2][1] - m[1][2]) / s;
            y = (m[0][2] - m[2][0]) / s;
            z = (m[1][0] - m[0][1]) / s;
            vector = new Vector3d(x, y, z);
            vector.Unitize();
            vector *= angle;
            return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, vector.X, vector.Y, vector.Z }; // return 180 deg rotation
        }

        public static Plane AxisAngleToPlane(double x, double y, double z, double vx, double vy, double vz)
        {
            var matrix = Transform.Identity;
            var vector = new Vector3d(vx, vy, vz);
            double angle = vector.Length;
            vector.Unitize();

            double c = Cos(angle);
            double s = Sin(angle);
            double t = 1.0 - c;

            matrix.M00 = c + vector.X * vector.X * t;
            matrix.M11 = c + vector.Y * vector.Y * t;
            matrix.M22 = c + vector.Z * vector.Z * t;

            double tmp1 = vector.X * vector.Y * t;
            double tmp2 = vector.Z * s;
            matrix.M10 = tmp1 + tmp2;
            matrix.M01 = tmp1 - tmp2;
            tmp1 = vector.X * vector.Z * t;
            tmp2 = vector.Y * s;
            matrix.M20 = tmp1 - tmp2;
            matrix.M02 = tmp1 + tmp2; tmp1 = vector.Y * vector.Z * t;
            tmp2 = vector.X * s;
            matrix.M21 = tmp1 + tmp2;
            matrix.M12 = tmp1 - tmp2;

            Plane plane = Plane.WorldXY;
            plane.Transform(matrix);
            plane.Origin = new Point3d(x, y, z);
            return plane;
        }

        public override double[] PlaneToNumbers(Plane plane)
        {
            // Plane originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
            Plane originPlane = Plane.WorldXY;
            plane.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));
            Point3d point = plane.Origin / 1000;
            plane.Origin = point;
            double[] axisAngle = PlaneToAxisAngle(plane, Plane.WorldXY);
            return axisAngle;
        }

        public override Plane NumbersToPlane(double[] numbers) => AxisAngleToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);

        public override double DegreeToRadian(double degree, int i, int group = 0)
        {
            return degree.ToRadians();
        }

        public KinematicSolution Kinematics(Target target, double[] prevJoints = null)
        {
            var kinematics = Kinematics(new Target[] { target }, new double[][] { prevJoints });
            return kinematics[0];
        }

        public override List<KinematicSolution> Kinematics(IEnumerable<Target> targets, IEnumerable<double[]> prevJoints = null)
        {
            var target = targets.First();
            var prevJoint = prevJoints?.First();
            var kinematics = new List<KinematicSolution>();
            var kinematic = Robot.Kinematics(target, prevJoint, this.BasePlane);
            var planes = kinematic.Planes.ToList();


            // Tool
            if (target.Tool != null)
            {
                Plane toolPlane = target.Tool.Tcp;
                toolPlane.Transform(planes[planes.Count - 1].ToTransform());
                planes.Add(toolPlane);
            }
            else
                planes.Add(planes[planes.Count - 1]);

            kinematic.Planes = planes.ToArray();
            kinematics.Add(kinematic);
            return kinematics;
        }

        internal override double Payload(int group)
        {
            return Robot.Payload;
        }

        internal override Joint[] GetJoints(int group)
        {
            return Robot.Joints.ToArray();
        }

        internal override List<List<List<string>>> Code(Program program) => new URScriptPostProcessor(this, program).Code;

        internal override void SaveCode(Program program, string folder)
        {
            if (!Directory.Exists(folder)) throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");
            if (program.Code == null) throw new NullReferenceException(" Program code not generated");

            string file = Path.Combine(folder, $"{program.Name}.URS");
            var joinedCode = string.Join("\r\n", program.Code[0].SelectMany(c => c));
            File.WriteAllText(file, joinedCode);
        }

        class URScriptPostProcessor
        {
            RobotUR robot;
            RobotCellUR cell;
            Program program;
            internal List<List<List<string>>> Code { get; }

            internal URScriptPostProcessor(RobotCellUR robotCell, Program program)
            {
                this.cell = robotCell;
                this.robot = cell.Robot as RobotUR;
                this.program = program;
                var groupCode = new List<List<string>> { Program() };
                this.Code = new List<List<List<string>>> { groupCode };
            }

            List<string> Program()
            {
                string indent = "  ";
                var code = new List<string>
                {
                    "def Program():"
                };

                // Attribute declarations

                foreach (var tool in program.Attributes.OfType<Tool>())
                {
                    Plane tcp = tool.Tcp;
                    Plane originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
                    tcp.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));
                    Point3d tcpPoint = tcp.Origin / 1000;
                    tcp.Origin = tcpPoint;
                    double[] axisAngle = PlaneToAxisAngle(tcp, Plane.WorldXY);

                    Point3d cog = tool.Centroid;
                    cog.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));
                    cog /= 1000;

                    code.Add(indent + $"{tool.Name}Tcp = p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}]");
                    code.Add(indent + $"{tool.Name}Weight = {tool.Weight:0.###}");
                    code.Add(indent + $"{tool.Name}Cog = [{cog.X:0.#####}, {cog.Y:0.#####}, {cog.Z:0.#####}]");
                }

                foreach (var speed in program.Attributes.OfType<Speed>())
                {
                    double linearSpeed = speed.TranslationSpeed / 1000;
                    code.Add(indent + $"{speed.Name} = {linearSpeed:0.#####}");
                }

                foreach (var zone in program.Attributes.OfType<Zone>())
                {
                    double zoneDistance = zone.Distance / 1000;
                    code.Add(indent + $"{zone.Name} = {zoneDistance:0.#####}");
                }

                foreach (var command in program.Attributes.OfType<Command>())
                {
                    string declaration = command.Declaration(program);
                    if (declaration != null && declaration.Length > 0)
                    {
                        declaration = indent + declaration;
                        //  declaration = indent + declaration.Replace("\n", "\n" + indent);
                        code.Add(declaration);
                    }
                }

                // Init commands

                foreach (var command in program.InitCommands)
                    code.Add(command.Code(program, Target.Default));

                Tool currentTool = null;

                // Targets

                foreach (var cellTarget in program.Targets)
                {
                    var programTarget = cellTarget.ProgramTargets[0];
                    var target = programTarget.Target;

                    if (currentTool == null || target.Tool != currentTool)
                    {
                        code.Add(Tool(target.Tool));
                        currentTool = target.Tool;
                    }

                    string moveText = null;
                    string zoneDistance = $"{target.Zone.Name:0.#####}";
                    // double zoneDistance = target.Zone.Distance / 1000;


                    if (programTarget.IsJointTarget || (programTarget.IsJointMotion && programTarget.ForcedConfiguration))
                    {
                        double[] joints = programTarget.IsJointTarget ? (programTarget.Target as JointTarget).Joints : programTarget.Kinematics.Joints;
                        double maxAxisSpeed = robot.Joints.Max(x => x.MaxSpeed);
                        double percentage = (cellTarget.DeltaTime > 0) ? cellTarget.MinTime / cellTarget.DeltaTime : 0.1;
                        double axisSpeed = percentage * maxAxisSpeed;
                        double axisAccel = target.Speed.AxisAccel;

                        string speed = null;
                        if (target.Speed.Time == 0)
                            speed = $"v={axisSpeed: 0.###}";
                        else
                            speed = $"t={target.Speed.Time: 0.###}";

                        moveText = $"  movej([{joints[0]:0.####}, {joints[1]:0.####}, {joints[2]:0.####}, {joints[3]:0.####}, {joints[4]:0.####}, {joints[5]:0.####}], a={axisAccel:0.####}, {speed}, r={zoneDistance})";
                    }
                    else
                    {
                        var cartesian = target as CartesianTarget;
                        var plane = cartesian.Plane;
                        plane.Transform(Transform.PlaneToPlane(Plane.WorldXY, target.Frame.Plane));
                        plane.Transform(Transform.PlaneToPlane(cell.BasePlane, Plane.WorldXY));
                        var axisAngle = cell.PlaneToNumbers(plane);

                        switch (cartesian.Motion)
                        {
                            case Motions.Joint:
                                {
                                    double maxAxisSpeed = robot.Joints.Min(x => x.MaxSpeed);
                                    double percentage = (cellTarget.DeltaTime > 0) ? cellTarget.MinTime / cellTarget.DeltaTime : 0.1;
                                    double axisSpeed = percentage * maxAxisSpeed;
                                    double axisAccel = target.Speed.AxisAccel;

                                    string speed = null;
                                    if (target.Speed.Time == 0)
                                        speed = $"v={axisSpeed: 0.###}";
                                    else
                                        speed = $"t={target.Speed.Time: 0.###}";

                                    moveText = $"  movej(p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}],a={axisAccel:0.#####},{speed},r={zoneDistance})";
                                    break;
                                }

                            case Motions.Linear:
                                {
                                    double linearSpeed = target.Speed.TranslationSpeed / 1000;
                                    double linearAccel = target.Speed.TranslationAccel / 1000;

                                    string speed = null;
                                    if (target.Speed.Time == 0)
                                        // speed = $"v={linearSpeed: 0.000}";
                                        speed = $"v={target.Speed.Name}";
                                    else
                                        speed = $"t={target.Speed.Time: 0.000}";

                                    moveText = $"  movel(p[{axisAngle[0]:0.#####}, {axisAngle[1]:0.#####}, {axisAngle[2]:0.#####}, {axisAngle[3]:0.#####}, {axisAngle[4]:0.#####}, {axisAngle[5]:0.#####}],a={linearAccel:0.#####},{speed},r={zoneDistance})";
                                    break;
                                }
                        }
                    }

                    foreach (var command in programTarget.Commands.Where(c => c.RunBefore))
                    {
                        string commands = command.Code(program, target);
                        commands = indent + commands;
                        code.Add(commands);
                    }

                    code.Add(moveText);

                    foreach (var command in programTarget.Commands.Where(c => !c.RunBefore))
                    {
                        string commands = command.Code(program, target);
                        commands = indent + commands;
                        code.Add(commands);
                    }
                }

                code.Add("end");
                return code;
            }

            string Tool(Tool tool)
            {
                string pos = $"  set_tcp({tool.Name}Tcp)";
                string mass = $"  set_payload({tool.Name}Weight, {tool.Name}Cog)";

                /*
                Plane tcp = tool.Tcp;
                Plane originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
                tcp.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));
                Point3d tcpPoint = tcp.Origin / 1000;
                double[] axisAngle = AxisAngle(tcp, Plane.WorldXY);

                Point3d cog = tool.Centroid;
                cog.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));
                cog /= 1000;

                string tcpString = $"p[{tcpPoint.X:0.00000}, {tcpPoint.Y:0.00000}, {tcpPoint.Z:0.00000}, {axisAngle[0]:0.0000}, {axisAngle[1]:0.0000}, {axisAngle[2]:0.0000}]";
                string cogString = $"[{cog.X:0.00000}, {cog.Y:0.00000}, {cog.Z:0.00000}]";
                string pos = $"  set_tcp({tcpString})";
                string mass = $"  set_payload({tool.Weight:0.000}, {cogString})";
                */
                return $"{pos}\n{mass}";
            }
        }
    }
}
