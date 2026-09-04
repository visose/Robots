namespace Robots;

class CustomKinematics(Custom custom) : MechanismKinematics(custom)
{
    protected override void SetPlanes(KinematicSolution solution, Target target) =>
        SetStartPlanes(solution);
}
