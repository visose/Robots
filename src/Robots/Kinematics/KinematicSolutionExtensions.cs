using Rhino.Geometry;

namespace Robots;

internal static class KinematicSolutionExtensions
{
    extension(IReadOnlyList<KinematicSolution> solutions)
    {
        internal double[] AllJoints() => solutions.FlattenToArray(solution => solution.Joints);

        internal Plane[] AllPlanes() => solutions.FlattenToArray(solution => solution.Planes);

        internal string[] AllErrors() => solutions.FlattenToArray(solution => solution.Errors);
    }
}
