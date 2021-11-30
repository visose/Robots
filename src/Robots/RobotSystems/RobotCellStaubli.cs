using System;
using System.Text;
using System.Collections.Generic;
using System.IO;
using Rhino.Geometry;
using static System.Math;
using static Robots.Util;

namespace Robots
{
    public class RobotCellStaubli : RobotCell
    {
        internal RobotCellStaubli(string name, List<MechanicalGroup> mechanicalGroups, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.Staubli, mechanicalGroups, io, basePlane, environment)
        {
        }

        public static Plane EulerToPlane(double x, double y, double z, double aDeg, double bDeg, double cDeg)
        {
            double a = aDeg.ToRadians();
            double b = bDeg.ToRadians();
            double c = cDeg.ToRadians();
            double ca = Cos(a);
            double sa = Sin(a);
            double cb = Cos(b);
            double sb = Sin(b);
            double cc = Cos(c);
            double sc = Sin(c);
            var tt = new Transform(1);
            tt[0, 0] = cb * cc; tt[0, 1] = ca * sc + sa * sb * cc; tt[0, 2] = sa * sc - ca * sb * cc;
            tt[1, 0] = -cb * sc; tt[1, 1] = ca * cc - sa * sb * sc; tt[1, 2] = sa * cc + ca * sb * sc;
            tt[2, 0] = sb; tt[2, 1] = -sa * cb; tt[2, 2] = ca * cb;

            var plane = tt.ToPlane();
            plane.Origin = new Point3d(x, y, z);
            return plane;
        }

        public static double[] PlaneToEuler(Plane plane)
        {
            Transform matrix = Transform.PlaneToPlane(Plane.WorldXY, plane);
            double a = Atan2(-matrix.M12, matrix.M22);
            double mult = 1.0 - matrix.M02 * matrix.M02;
            if (Abs(mult) < UnitTol) mult = 0.0;
            double b = Atan2(matrix.M02, Sqrt(mult));
            double c = Atan2(-matrix.M01, matrix.M00);

            if (matrix.M02 < (-1.0 + UnitTol))
            {
                a = Atan2(matrix.M21, matrix.M11);
                b = -PI / 2;
                c = 0;
            }
            else if (matrix.M02 > (1.0 - UnitTol))
            {
                a = Atan2(matrix.M21, matrix.M11);
                b = PI / 2;
                c = 0;
            }

            return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, a.ToDegrees(), b.ToDegrees(), c.ToDegrees() };
        }

        public override double[] PlaneToNumbers(Plane plane) => PlaneToEuler(plane);
        public override Plane NumbersToPlane(double[] numbers) => EulerToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);

        internal override void SaveCode(IProgram program, string folder)
        {
            if (!Directory.Exists(folder))
                throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");

            if (program.Code is null)
                throw new NullReferenceException(" Program code not generated");

            var programDir = Path.Combine(folder, program.Name);
            Directory.CreateDirectory(programDir);

            for (int i = 0; i < program.Code.Count; i++)
            {
                string group = MechanicalGroups[i].Name;
                //string programName = $"{program.Name}_{group}";
                string programName = $"{program.Name}";

                for (int j = 0; j < program.Code[i].Count; j++)
                {
                    string name;

                    switch (j)
                    {
                        case 0:
                            name = $"{programName}.pjx"; break;
                        case 1:
                            name = $"{programName}.dtx"; break;
                        case 2:
                            name = "start.pgx"; break;
                        case 3:
                            name = "stop.pgx"; break;
                        default:
                            name = $"{programName}_{j - 4:000}.pgx"; break;
                    }

                    string file = Path.Combine(programDir, name);
                    var joinedCode = string.Join("\r\n", program.Code[i][j]);
                    //File.WriteAllText(file, joinedCode);

                    var utf8WithoutBom = new UTF8Encoding(true);
                    var writer = new StreamWriter(file, false, utf8WithoutBom);
                    writer.WriteLine(joinedCode);
                    writer.Close();
                }
            }
        }

        internal override List<List<List<string>>> Code(Program program) => new VAL3PostProcessor(this, program).Code;
    }
}