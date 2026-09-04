namespace Robots;

public abstract class SingleGroupSystem : IndustrialSystem
{
    public MechanicalGroup MechanicalGroup => MechanicalGroups[0];
    public RobotArm Robot => MechanicalGroup.Robot;

    internal SingleGroupSystem(SystemAttributes attributes, MechanicalGroup mechanicalGroup)
        : base(attributes, [mechanicalGroup]) { }
}
