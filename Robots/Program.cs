using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace Robots
{
    public class Program
    {
        public string Name { get; }
        public Robot Robot { get; }
        public List<Target> Targets { get; set; }
        public Commands.Group InitCommands { get; set; }
        public List<string> Code { get; set; }

        public Program(string name, Robot robot, IEnumerable<Target> targets = null, Commands.Group initCommands = null)
        {
            this.Name = name;
            this.Robot = robot;
            this.Targets = (targets != null) ? targets.ToList() : new List<Target>();
            this.InitCommands = (initCommands != null) ? initCommands : new Commands.Group();
        }

        public void GenerateCode()
        {
            CheckProgram();
            Code = Robot.Code(this);
        }

        public void Save(string folder)
        {
            if (!Directory.Exists(folder)) throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");
            if (Code == null) throw new NullReferenceException(" Program code not generated");
            string file = $@"{folder}\{this.Name}.{Robot.Extension}";
            var joinedCode = string.Join("\r\n", Code);
            File.WriteAllText(file, joinedCode);
        }

        void CheckProgram()
        {
            for (int i = 0; i < Targets.Count; i++)
            {
                if (Targets[i].Tool == null) throw new NullReferenceException($" Tool not set in target {i}");
                if (Targets[i].Speed == null) throw new NullReferenceException($" Speed not set in target {i}");
                if (Targets[i].Zone == null) throw new NullReferenceException($" Zone not set in target {i}");
            }
        }

        public int CheckKinematics(out List<string> errors)
        {
         errors = new List<string>();

            for(int i=0;i<Targets.Count;i++)
            {
                var kinematics = Robot.Kinematics(Targets[i], false);
                if (kinematics.Errors.Count > 0)
                {
                    errors.Add($"Errors found in target {i}");
                    errors.AddRange(kinematics.Errors);
                    return i;
                }         
            }
            errors.Add("No errors found");
            return -1;
        }

        public override string ToString() => $"Program ({Name} with {Targets.Count} targets)";
    }
}