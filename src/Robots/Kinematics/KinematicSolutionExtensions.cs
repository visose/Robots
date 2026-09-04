using Rhino.Geometry;

namespace Robots;

static class KinematicSolutionExtensions
{
    extension(IReadOnlyList<KinematicSolution> solutions)
    {
        public double[][] JointSets(double[][]? result = null)
        {
            result ??= new double[solutions.Count][];
            ArgumentOutOfRangeException.ThrowIfNotEqual(result.Length, solutions.Count, nameof(result));

            for (int i = 0; i < result.Length; i++)
                result[i] = solutions[i].Joints;

            return result;
        }

        public double[] AllJoints() => solutions.FlattenToArray(solution => solution.Joints);

        public Plane[] AllPlanes() => solutions.FlattenToArray(solution => solution.Planes);

        public string[] AllErrors() => solutions.FlattenToArray(solution => solution.Errors);
    }
}
