using Rhino.Geometry;

namespace Robots;

internal static class KinematicSolutionExtensions
{
    extension(IReadOnlyList<KinematicSolution> solutions)
    {
        internal double[][] JointSets(double[][]? result = null)
        {
            result ??= new double[solutions.Count][];
            ArgumentOutOfRangeException.ThrowIfNotEqual(result.Length, solutions.Count, nameof(result));

            for (int i = 0; i < result.Length; i++)
                result[i] = solutions[i].Joints;

            return result;
        }

        internal double[] AllJoints() => solutions.FlattenToArray(solution => solution.Joints);

        internal Plane[] AllPlanes() => solutions.FlattenToArray(solution => solution.Planes);

        internal string[] AllErrors() => solutions.FlattenToArray(solution => solution.Errors);
    }
}
