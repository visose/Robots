using System.Collections.Generic;

namespace Robots
{
    public class CustomProgram : IProgram
    {
        public string Name { get; }
        public RobotSystem RobotSystem { get; }
        public List<List<List<string>>>? Code { get; }
        public List<int> MultiFileIndices { get; }

        public CustomProgram(string name, RobotSystem robotSystem, List<int> multiFileIndices, List<List<List<string>>> code)
        {
            Name = name;
            RobotSystem = robotSystem;
            Code = code;
            MultiFileIndices = multiFileIndices;
        }

        public void Save(string folder) => RobotSystem.SaveCode(this, folder);

        public override string ToString() => $"Program ({Name} with custom code)";        
    }
}