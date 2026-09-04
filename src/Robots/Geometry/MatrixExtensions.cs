namespace Robots;

static class MatrixExtensions
{
    extension(double[,] matrix)
    {
        public double[,] Mult(double[,] other)
        {
            int rows = matrix.GetLength(0);
            int shared = matrix.GetLength(1);
            int columns = other.GetLength(1);

            if (shared != other.GetLength(0))
                throw new ArgumentException("Matrices have incompatible dimensions.");

            var result = new double[rows, columns];

            for (int row = 0; row < rows; row++)
            {
                for (int column = 0; column < columns; column++)
                {
                    double value = 0;

                    for (int index = 0; index < shared; index++)
                        value += matrix[row, index] * other[index, column];

                    result[row, column] = value;
                }
            }

            return result;
        }

        public double[,] Transpose()
        {
            int rows = matrix.GetLength(0);
            int columns = matrix.GetLength(1);
            var result = new double[columns, rows];

            for (int row = 0; row < rows; row++)
            {
                for (int column = 0; column < columns; column++)
                    result[column, row] = matrix[row, column];
            }

            return result;
        }
    }

    extension(double[] vector)
    {
        public double[] Mult(double[,] matrix)
        {
            int rows = matrix.GetLength(0);
            int columns = matrix.GetLength(1);

            if (vector.Length != rows)
                throw new ArgumentException("Matrices have incompatible dimensions.");

            var result = new double[columns];

            for (int column = 0; column < columns; column++)
            {
                double value = 0;

                for (int index = 0; index < vector.Length; index++)
                    value += vector[index] * matrix[index, column];

                result[column] = value;
            }

            return result;
        }
    }
}
