using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.RapidDomain;
using ABB.Robotics.Controllers.Discovery;

namespace Robots
{
    public class RemoteAbb : IRemote
    {        
        readonly RobotCellAbb _cell;
        Controller? _controller;

        public string? IP { get; set; }
        public List<string> Log { get; } = new List<string>();
        public bool Connected => _controller != null && _controller.Connected;

        internal RemoteAbb(RobotCellAbb cell)
        {
            _cell = cell;
        }

        public void Play() => Command(StartCommand);
        public void Pause() => Command(StopCommand);
        public void Upload(IProgram p) => Command(() => UploadCommand(p));

        void AddLog(string text)
        {
            Log.Insert(0, $"{DateTime.Now.ToLongTimeString()} - {text}");
        }

       public void Connect()
        {
            Disconnect();
            NetworkScanner scanner = new NetworkScanner();
            scanner.Scan();

            var controllerInfo =
                     (IP != null) ?
                     scanner.Controllers.FirstOrDefault(c => c.IPAddress.ToString() == IP) :
                     scanner.Controllers.FirstOrDefault();

            if (controllerInfo is null)
            {
                AddLog("Controller not found.");
                return;
            }

            if (controllerInfo.Availability != Availability.Available)
            {
                AddLog("Controller not available.");
                return;
            }

            _controller = ControllerFactory.CreateFrom(controllerInfo);
            _controller.Logon(UserInfo.DefaultUser);

            // string isVirtual = controllerInfo.IsVirtual ? "Virtual" : "Real";
            // AddLog($"Connecting to {controllerInfo.Name} ({isVirtual})");

            if (!Connected)
            {
                AddLog("Couldn't connect to controller.");
                return;
            }
        }

       public void Disconnect()
        {
            if (_controller != null)
            {
                // AddLog("Disconnecting from controller.");
                // var task = _controller.Rapid.GetTasks().First();
                // task.Stop();
                _controller.Logoff();
                _controller.Dispose();
                _controller = null;
            }
        }

        void Command(Func<string> command)
        {
            Connect();
            if (Connected) AddLog(command());
            Disconnect();
        }

        [Obsolete]
        string StartCommand()
        {
            if (_controller is null)
                return "Controller is null.";

            if (_controller.OperatingMode != ControllerOperatingMode.Auto)
                return "Controller not set in automatic.";

            if (_controller.State != ControllerState.MotorsOn)
                return "Motors not on.";

            using (Mastership master = Mastership.Request(_controller.Rapid))
            {
                _controller.Rapid.Start(RegainMode.Continue, ExecutionMode.Continuous, ExecutionCycle.Once, StartCheck.CallChain);
            }

            return "Program started.";
        }

        string StopCommand()
        {
            if (_controller is null)
                return "Controller is null.";

            if (_controller.OperatingMode != ControllerOperatingMode.Auto)
                return "Controller not set in automatic.";

            using (Mastership master = Mastership.Request(_controller.Rapid))
            {
                _controller.Rapid.Stop(StopMode.Instruction);
            }

            return "Program stopped.";
        }

        string UploadCommand(IProgram program)
        {
            if (_controller is null)
                return "Controller is null.";

            try
            {
                // task.Stop(StopMode.Instruction);

                string tempPath = Path.Combine(Util.LibraryPath, "temp");
                if (Directory.Exists(tempPath)) Directory.Delete(tempPath, true);
                Directory.CreateDirectory(tempPath);

                program.Save(tempPath);
                string localFolder = Path.Combine(tempPath, program.Name);
                string robotFolder = $@"{_controller.FileSystem.RemoteDirectory}\{program.Name}";
                string filePath = string.Empty;

                if (!_controller.IsVirtual)
                {
                    _controller.AuthenticationSystem.DemandGrant(Grant.WriteFtp);
                    _controller.FileSystem.PutDirectory(localFolder, program.Name, true);
                    filePath = $@"{robotFolder}\{program.Name}_T_ROB1.pgf";
                }
                else
                {
                    filePath = $@"{localFolder}\{program.Name}_T_ROB1.pgf";
                }

                using (Mastership master = Mastership.Request(_controller.Rapid))
                {
                    var task = _controller.Rapid.GetTasks().First();
                    task.DeleteProgram();
                    int count = 0;
                    while (count++ < 10)
                    {
                        System.Threading.Thread.Sleep(100);
                        try
                        {
                            _controller.AuthenticationSystem.DemandGrant(Grant.LoadRapidProgram);
                            if (task.LoadProgramFromFile(filePath, RapidLoadMode.Replace))
                                return $"Program {program.Name} uploaded to {_controller.Name}.";
                        }
                        catch (Exception e)
                        {
                            Log.Insert(0,$"Retrying {count}: {e}");
                        }
                    }

                    //    task.ResetProgramPointer();
                }
            }
            catch (Exception e)
            {
                return $"Error uploading: {e}";
            }

            return "Unknown error";
        }

        public JointTarget GetPosition()
        {
            if (!Connected) Connect();// return Target.Default as JointTarget;

            if (_controller is null)
                throw new NullReferenceException(" Controller is null.");

            var joints = _controller.MotionSystem.ActiveMechanicalUnit.GetPosition();
            var values = new double[] { joints.RobAx.Rax_1, joints.RobAx.Rax_2, joints.RobAx.Rax_3, joints.RobAx.Rax_4, joints.RobAx.Rax_5, joints.RobAx.Rax_6 };

            for (int i = 0; i < 6; i++)
                values[i] = RobotAbb.ABBDegreeToRadian(values[i], i);

            var target = new JointTarget(values);
            return target;
        }
    }
}
