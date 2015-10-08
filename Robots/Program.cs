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
        public Commands.List InitCommands { get; set; }

        readonly Tool defaultTool = new Tool();
        readonly Speed defaultSpeed = new Speed();
        readonly Zone defaultZone = new Zone();

        public Program(string name, Robot robot, List<Target> targets = null, Commands.List initCommands = null)
        {
            this.Name = name;
            this.Robot = robot;
            this.Targets = (targets != null) ? targets : new List<Target>();
            this.InitCommands = (initCommands != null) ? initCommands : new Commands.List();
        }

        public StringBuilder Code() => Robot.Code(this);

        public void Save(string folder)
        {
            string file = $@"{folder}\{this.Name}.{Robot.Extension}";
            var code = this.Code();
            File.WriteAllText(file, code.ToString());
        }
    }
}