namespace Robots;

public class CustomProgram(string name, RobotSystem robotSystem, IReadOnlyList<int> multiFileIndices, List<List<List<string>>> code) : IProgram
{
    public string Name { get; } = name;
    public RobotSystem RobotSystem { get; } = robotSystem;
    public List<List<List<string>>>? Code { get; } = RobotSystem.SplitCodeLines(code);
    public IReadOnlyList<int> MultiFileIndices { get; } = Array.AsReadOnly([.. multiFileIndices]);
    public bool HasSimulation => false;

    public void Save(string folder)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(folder);
        RobotSystem.SaveCode(this, folder);
    }

    public override string ToString() => $"Program ({Name} with custom code)";
}
