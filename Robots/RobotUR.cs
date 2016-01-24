using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using static System.Math;

namespace Robots
{
    class RobotUR : Robot
    {
        public RemoteConnection Remote { get; } = new RemoteConnection();
        internal RobotUR(string model, double payload, Plane basePlane, Mesh baseMesh, Joint[] joints, RobotIO io) : base(model, payload, basePlane, baseMesh, joints, io)
        {
            this.Manufacturer = Manufacturers.UR;
            this.Extension = "URS";
        }

        public override KinematicSolution Kinematics(Target target, bool calculateMeshes = true) => new OffsetWristKinematics(target, this, calculateMeshes);
        internal override List<string> Code(Program program) => new URScriptPostProcessor(this, program).Code;
        protected override double[] GetStartPose() => new double[] { 0, -PI / 2, 0, -PI / 2, 0, 0 };
        public override double DegreeToRadian(double degree, int i) => degree * (PI / 180);
        public override double RadianToDegree(double radian, int i) => radian * (180 / PI);

        /// <summary>
        /// Code lifted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
        /// </summary>
        /// <param name="plane"></param>
        /// <param name="originPlane"></param>
        /// <returns></returns>
        public static double[] AxisAngle(Plane plane, Plane originPlane)
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
                //  in leading diagonaland zero in other terms
                if ((Abs(m[0][1] + m[1][0]) < epsilon2)
                  && (Abs(m[0][2] + m[2][0]) < epsilon2)
                  && (Abs(m[1][2] + m[2][1]) < epsilon2)
                && (Abs(m[0][0] + m[1][1] + m[2][2] - 3) < epsilon2))
                {
                    // this singularity is identity matrix so angle = 0
                    return new double[] { 0, 0, 0 }; // zero angle, arbitrary axis
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
                return new double[] { vector.X, vector.Y, vector.Z }; // return 180 deg rotation
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
            return new double[] { vector.X, vector.Y, vector.Z }; // return 180 deg rotation
        }

        public class RemoteConnection
        {
            TcpClient client = new TcpClient();
            public string IP { get; set; }
            public int Port { get; set; } = 30002;
            public List<string> Log { get; } = new List<string>();
            public bool IsConnected => client.Connected;

            public void Connect()
            {
                if (IP == null) throw new NullReferenceException(" IP for UR robot has not been set");
                if (IsConnected) Disconnect();
                client.Connect(IP, Port);
                Log.Add((IsConnected) ? "Connected" : "Not able to connect");
            }

            public void Disconnect()
            {
                if (IsConnected)
                {
                    client.Close();
                    client = new TcpClient();
                    Log.Add("Disconnected");
                }
            }

            public void Send(string message, bool verbose = false)
            {
                if (!IsConnected)
                {
                    Log.Add("Can't send message, not connected");
                    return;
                }

                message += '\n';
                var asen = new System.Text.ASCIIEncoding();
                byte[] byteArray = asen.GetBytes(message);
                NetworkStream stream = client.GetStream();
                stream.Write(byteArray, 0, byteArray.Length);

                if (verbose)
                {
                    string firstLine = message.Substring(0, message.IndexOf('\n'));
                    if (firstLine.Length + 1 < message.Length) firstLine += " ...";
                    Log.Add($"Sending: {firstLine}");
                }
            }

            public void UploadProgram(Program program)
            {
                var joinedCode = string.Join("\n", program.Code);
                Send(joinedCode);
            }

            public void Pause() => Send("pause program");
            public void Play() => Send("resume program");

        }

        class URScriptPostProcessor
        {
            RobotUR robot;
            Program program;
            internal List<string> Code { get; }

            internal URScriptPostProcessor(RobotUR robot, Program program)
            {
                this.robot = robot;
                this.program = program;
                this.Code = PostProcessor();
            }

            List<string> PostProcessor()
            {
                var code = new List<string>();
                code.Add("def Program():");

                string initCommands = program.InitCommands.Code(program.Robot, Target.Default);
                if (initCommands.Length > 0)
                    code.Add(initCommands);

                Tool currentTool = null;

                foreach (ProgramTarget target in program.Targets)
                {
                    if (currentTool == null || target.Tool != currentTool)
                    {
                        code.Add(Tool(target.Tool));
                        currentTool = target.Tool;
                    }

                    string moveText = null;

                    if (target.IsJointTarget || (target.Motion == Target.Motions.Joint && target.ForcedConfiguration))
                    {
                        double[] joints = target.Joints;
                        double maxAxisSpeed = robot.Joints.Max(x => x.MaxSpeed);
                        double percentage = (target.Speed.AxisSpeed > 0) ? target.Speed.AxisSpeed : (target.Time > 0) ? target.MinTime / target.Time : 0.1;
                        double axisSpeed = percentage * maxAxisSpeed;
                        double axisAccel = target.Speed.AxisAccel;
                        double zone = target.Zone.Distance / 1000;
                        moveText = $"  movej([{joints[0]:0.0000}, {joints[1]:0.0000}, {joints[2]:0.0000}, {joints[3]:0.0000}, {joints[4]:0.0000}, {joints[5]:0.0000}], a={axisAccel:0.0000}, v={axisSpeed:0.000}, r={zone:0.00000})";
                    }
                    else
                    {
                        var plane = target.Plane;
                        plane.Transform(Transform.PlaneToPlane(robot.basePlane, Plane.WorldXY));
                        var origin = target.Plane.Origin / 1000;
                        var axisAngle = AxisAngle(plane, Plane.WorldXY);

                        switch (target.Motion)
                        {
                            case Target.Motions.Joint:
                                {
                                    double maxAxisSpeed = robot.Joints.Max(x => x.MaxSpeed);
                                    double percentage = (target.Speed.AxisSpeed > 0) ? target.Speed.AxisSpeed : (target.Time > 0) ? target.MinTime / target.Time : 0.1;
                                    double axisSpeed = percentage * maxAxisSpeed;
                                    double axisAccel = target.Speed.AxisAccel;
                                    double zone = target.Zone.Distance / 1000;
                                    moveText = $"  movej(p[{origin.X:0.00000}, {origin.Y:0.00000}, {origin.Z:0.00000}, {axisAngle[0]:0.0000}, {axisAngle[1]:0.0000}, {axisAngle[2]:0.0000}],a={axisAccel:0.00000},v={axisSpeed:0.00000},r={zone:0.00000})";
                                    break;
                                }

                            case Target.Motions.Linear:
                                {
                                    double linearSpeed = target.Speed.TranslationSpeed / 1000;
                                    double linearAccel = target.Speed.TranslationAccel / 1000;
                                    double zone = target.Zone.Distance / 1000;
                                    moveText = $"  movel(p[{origin.X:0.00000}, {origin.Y:0.00000}, {origin.Z:0.00000}, {axisAngle[0]:0.0000}, {axisAngle[1]:0.0000}, {axisAngle[2]:0.0000}],a={linearAccel:0.00000},v={linearSpeed:0.00000},r={zone:0.00000})";
                                    break;
                                }
                        }
                    }

                    code.Add(moveText);

                    string commands = target.Commands.Code(program.Robot, target);
                    if (commands.Length > 0)
                    {
                        string indent = "  ";
                        commands = indent + commands.Replace("\n", "\n" + indent);
                        code.Add(commands);
                    }
                }

                code.Add("end");
                return code;
            }

            string Tool(Tool tool)
            {
                Plane tcp = tool.Tcp;
                //Plane originPlane = new Plane(Point3d.Origin, -Vector3d.YAxis, Vector3d.XAxis);
                Plane originPlane = new Plane(Point3d.Origin, Vector3d.YAxis, -Vector3d.XAxis);
                tcp.Transform(Transform.PlaneToPlane(Plane.WorldXY, originPlane));
                Point3d origin = tcp.Origin / 1000;
                double[] axisAngle = AxisAngle(tcp, Plane.WorldXY);
                string pos = $"  set_tcp(p[{origin.X:0.00000}, {origin.Y:0.00000}, {origin.Z:0.00000}, {axisAngle[0]:0.0000}, {axisAngle[1]:0.0000}, {axisAngle[2]:0.0000}])";
                string mass = $"  set_payload({tool.Weight:0.000})";
                return $"{pos}\n{mass}";
            }
        }
    }
}
