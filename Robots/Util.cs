using System;
using static System.Math;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;

namespace Robots
{
    static class Util
    {
         public const double Tol = 0.001;

        static public Plane ToPlane(double[,] matrix)
        {
            var transform = new Transform();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    transform[i, j] = matrix[i, j];

            Plane plane = Plane.WorldXY;
            plane.Transform(transform);
            return plane;
        }

        static public double[] DegreesToRadians(double[] ds)
        {
            double[] rd = new double[6];
            for (int i = 0; i < 6; i++)
                rd[i] = ds[i] * PI / 180;
            rd[2] -= 0.5*PI;
            rd[5] += PI;
            for (int i = 0; i < 6; i++)
                rd[i] = -rd[i];
            return rd;
        }

        static public double DegreeToRadian(double degree, int i)
        {
                double radian =degree * PI / 180;
            if(i==2) radian -= 0.5*PI;
            if(i==5) radian += PI;
            radian = -radian;
            return radian;
        }

        static public double[] RadiansToDegrees(double[] ds)
        {
            double[] rd = new double[6];
            for (int i = 0; i < 6; i++)
                rd[i] = -ds[i];
            rd[2] += 0.5*PI;
            rd[5] -= PI;
            for (int i = 0; i < 6; i++)
                rd[i] = rd[i] * 180 / PI;
            return rd;
        }


        public static double[] mul34(double[][] a, double[] b)
        {
            double[] re = new double[3];
            for (int i = 0; i < 3; i++)
                re[i] = a[i][0] * b[0] + a[i][1] * b[1] + a[i][2] * b[2] + a[i][3];
            return re;
        }

        public static double[][] mul34(double[][] a, double[][] b)
        { 
            double[][] re = new double[3][];
            for (int i = 0; i < 3; i++)
                re[i] = new double[4];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    double b3j = (j == 3 ? 1 : 0);
                    re[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b3j;
                }
            }
            return re;
        }

        public static double[][] inverse34(double[][] m)
        { 
            double[][] v = new double[3][];
            for (int i=0;i<3;i++)
                v[i] = new double[4];

            v[0][0] = -m[1][2] * m[2][1] + m[1][1] * m[2][2];
            v[0][1] = m[0][2] * m[2][1] - m[0][1] * m[2][2];
            v[0][2] = -m[0][2] * m[1][1] + m[0][1] * m[1][2];
            v[0][3] = m[0][3] * m[1][2] * m[2][1] - m[0][2] * m[1][3] * m[2][1] - m[0][3] * m[1][1] * m[2][2] + m[0][1] * m[1][3] * m[2][2] + m[0][2] * m[1][1] * m[2][3] - m[0][1]
                    * m[1][2] * m[2][3];
            v[1][0] = m[1][2] * m[2][0] - m[1][0] * m[2][2];
            v[1][1] = -m[0][2] * m[2][0] + m[0][0] * m[2][2];
            v[1][2] = m[0][2] * m[1][0] - m[0][0] * m[1][2];
            v[1][3] = m[0][2] * m[1][3] * m[2][0] - m[0][3] * m[1][2] * m[2][0] + m[0][3] * m[1][0] * m[2][2] - m[0][0] * m[1][3] * m[2][2] - m[0][2] * m[1][0] * m[2][3] + m[0][0]
                    * m[1][2] * m[2][3];
            v[2][0] = -m[1][1] * m[2][0] + m[1][0] * m[2][1];
            v[2][1] = m[0][1] * m[2][0] - m[0][0] * m[2][1];
            v[2][2] = -m[0][1] * m[1][0] + m[0][0] * m[1][1];
            v[2][3] = m[0][3] * m[1][1] * m[2][0] - m[0][1] * m[1][3] * m[2][0] - m[0][3] * m[1][0] * m[2][1] + m[0][0] * m[1][3] * m[2][1] + m[0][1] * m[1][0] * m[2][3] - m[0][0]
                    * m[1][1] * m[2][3];
            return v;
        }

        public static double dist(double[] a, double[] b)
        {
            double r = 0;
            for (int i = 0; i < a.Length; i++)
                r += (a[i] - b[i]) * (a[i] - b[i]);
            return Sqrt(r);
        }

    }
}



