namespace Robots;

class FanucPostProcessor
{
    readonly RobotCellFanuc _cell;
    readonly Program _program;
    internal List<List<List<string>>> Code { get; }

    internal FanucPostProcessor(RobotCellFanuc robotCell, Program program)
    {
        _cell = robotCell;
        _program = program;
        Code = new List<List<List<string>>>();
        /*
         * TODO: Implement...
         */

        throw new NotImplementedException(" Fanuc post-processor not implemented.");
    }
}
