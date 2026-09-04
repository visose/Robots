namespace Robots;

public interface IPostProcessor
{
    CommandFormatter Commands => CommandFormatter.Empty;

    void Validate(Program program, IReadOnlyList<ProgramTarget> targets) =>
        PostProcessorUtil.RejectProcessMotions(program, targets);

    List<List<List<string>>> GetCode(RobotSystem system, Program program);

    void Save(IProgram program, string folder) =>
        throw new NotSupportedException($"{GetType().Name} does not support saving programs.");
}
