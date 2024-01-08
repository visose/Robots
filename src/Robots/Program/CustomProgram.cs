namespace Robots;

public class CustomProgram(string name, RobotSystem robotSystem, List<int> multiFileIndices, List<List<List<string>>> code) : IProgram
{
    public string Name { get; } = name;
    public RobotSystem RobotSystem { get; } = robotSystem;
    public List<List<List<string>>>? Code { get; } = code;
    public List<int> MultiFileIndices { get; } = multiFileIndices;
    public bool HasSimulation => false;

    public void Save(string folder) => RobotSystem.SaveCode(this, folder);

    public override string ToString() => $"Program ({Name} with custom code)";
}
