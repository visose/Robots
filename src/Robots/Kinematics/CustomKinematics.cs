using Rhino.Geometry;

namespace Robots;

class CustomKinematics : MechanismKinematics
{
    internal CustomKinematics(Custom custom)
        : base(custom) { }

    protected override void SetPlanes(KinematicSolution solution, Target target) =>
        SetStartPlanes(solution);
}
