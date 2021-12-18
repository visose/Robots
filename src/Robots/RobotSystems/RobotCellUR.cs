﻿using Rhino.Geometry;
using static System.Math;

namespace Robots;

public class RobotCellUR : RobotSystem
{
    public RobotUR Robot { get; }    
    // public URRealTime URRealTime { get; set; }

    internal RobotCellUR(string name, RobotUR robot, IO io, Plane basePlane, Mesh? environment) : base(name, Manufacturers.UR, io, basePlane, environment)
    {
        Remote = new RemoteUR();
        Robot = robot;
        DisplayMesh.Append(robot.DisplayMesh);
        DisplayMesh.Transform(BasePlane.ToTransform());
    }

    /// <summary>
    /// Code lifted from http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
    /// </summary>
    /// <param name="plane"></param>
    /// <param name="originPlane"></param>
    /// <returns></returns>
    public static double[] PlaneToAxisAngle(ref Plane plane)
    {
        Vector3d vector;
        var t = plane.ToTransform();

        double angle, x, y, z; // variables for result
        double epsilon = 0.01; // margin to allow for rounding errors
        double epsilon2 = 0.1; // margin to distinguish between 0 and 180 degrees
                               // optional check that input is pure rotation, 'isRotationMatrix' is defined at:
                               // http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/
                               // assert isRotationMatrix(m) : "not valid rotation matrix";// for debugging
        if ((Abs(t.M01 - t.M10) < epsilon)
          && (Abs(t.M02 - t.M20) < epsilon)
        && (Abs(t.M12 - t.M21) < epsilon))
        {
            // singularity found
            // first check for identity matrix which must have +1 for all terms
            //  in leading diagonal and zero in other terms
            if ((Abs(t.M01 + t.M10) < epsilon2)
              && (Abs(t.M02 + t.M20) < epsilon2)
              && (Abs(t.M12 + t.M21) < epsilon2)
            && (Abs(t.M00 + t.M11 + t.M22 - 3) < epsilon2))
            {
                // this singularity is identity matrix so angle = 0
                return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, 0, 0, 0 }; // zero angle, arbitrary axis
            }
            // otherwise this singularity is angle = 180
            angle = PI;
            double xx = (t.M00 + 1) / 2;
            double yy = (t.M11 + 1) / 2;
            double zz = (t.M22 + 1) / 2;
            double xy = (t.M01 + t.M10) / 4;
            double xz = (t.M02 + t.M20) / 4;
            double yz = (t.M12 + t.M21) / 4;
            if ((xx > yy) && (xx > zz))
            { // m.M00 is the largest diagonal term
                if (xx < epsilon)
                {
                    x = 0;
                    y = 0.7071;
                    z = 0.7071;
                }
                else
                {
                    x = Sqrt(xx);
                    y = xy / x;
                    z = xz / x;
                }
            }
            else if (yy > zz)
            { // m.M11 is the largest diagonal term
                if (yy < epsilon)
                {
                    x = 0.7071;
                    y = 0;
                    z = 0.7071;
                }
                else
                {
                    y = Sqrt(yy);
                    x = xy / y;
                    z = yz / y;
                }
            }
            else
            { // m.M22 is the largest diagonal term so base result on this
                if (zz < epsilon)
                {
                    x = 0.7071;
                    y = 0.7071;
                    z = 0;
                }
                else
                {
                    z = Sqrt(zz);
                    x = xz / z;
                    y = yz / z;
                }
            }
            vector = new Vector3d(x, y, z);
            vector.Unitize();
            vector *= angle;
            return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, vector.X, vector.Y, vector.Z }; // return 180 deg rotation
        }
        // as we have reached here there are no singularities so we can handle normally
        double s = Sqrt((t.M21 - t.M12) * (t.M21 - t.M12)
          + (t.M02 - t.M20) * (t.M02 - t.M20)
          + (t.M10 - t.M01) * (t.M10 - t.M01)); // used to normalise
        if (Abs(s) < 0.001) s = 1;
        // prevent divide by zero, should not happen if matrix is orthogonal and should be
        // caught by singularity test above, but I've left it in just in case
        angle = Acos((t.M00 + t.M11 + t.M22 - 1) / 2);
        x = (t.M21 - t.M12) / s;
        y = (t.M02 - t.M20) / s;
        z = (t.M10 - t.M01) / s;
        vector = new Vector3d(x, y, z);
        vector.Unitize();
        vector *= angle;
        return new double[] { plane.OriginX, plane.OriginY, plane.OriginZ, vector.X, vector.Y, vector.Z }; // return 180 deg rotation
    }

    public static Plane AxisAngleToPlane(double x, double y, double z, double vx, double vy, double vz)
    {
        var matrix = Transform.Identity;
        var vector = new Vector3d(vx, vy, vz);
        double angle = vector.Length;
        vector.Unitize();

        double c = Cos(angle);
        double s = Sin(angle);
        double t = 1.0 - c;

        matrix.M00 = c + vector.X * vector.X * t;
        matrix.M11 = c + vector.Y * vector.Y * t;
        matrix.M22 = c + vector.Z * vector.Z * t;

        double tmp1 = vector.X * vector.Y * t;
        double tmp2 = vector.Z * s;
        matrix.M10 = tmp1 + tmp2;
        matrix.M01 = tmp1 - tmp2;
        tmp1 = vector.X * vector.Z * t;
        tmp2 = vector.Y * s;
        matrix.M20 = tmp1 - tmp2;
        matrix.M02 = tmp1 + tmp2; tmp1 = vector.Y * vector.Z * t;
        tmp2 = vector.X * s;
        matrix.M21 = tmp1 + tmp2;
        matrix.M12 = tmp1 - tmp2;

        //Plane plane = Plane.WorldXY;
        //plane.Transform(matrix);
        var plane = matrix.ToPlane();
        plane.Origin = new Point3d(x, y, z);
        return plane;
    }

    public override double[] PlaneToNumbers(Plane plane)
    {
        Point3d point = plane.Origin / 1000.0;
        plane.Origin = point;
        double[] axisAngle = PlaneToAxisAngle(ref plane);
        return axisAngle;
    }

    public override Plane NumbersToPlane(double[] numbers) => AxisAngleToPlane(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5]);

    public override double DegreeToRadian(double degree, int i, int group = 0)
    {
        return degree.ToRadians();
    }

    public KinematicSolution Kinematics(Target target, double[]? prevJoints = null)
    {
        var kinematics = Kinematics(new Target[] { target }, new double[]?[] { prevJoints });
        return kinematics[0];
    }

    public override List<KinematicSolution> Kinematics(IEnumerable<Target> targets, IEnumerable<double[]?>? prevJoints = null)
    {
        var target = targets.First();
        var prevJoint = prevJoints?.First();
        var kinematics = new List<KinematicSolution>();
        var kinematic = Robot.Kinematics(target, prevJoint, BasePlane);
        var planes = kinematic.Planes.ToList();


        // Tool
        if (target.Tool is not null)
        {
            Plane toolPlane = target.Tool.Tcp;
            toolPlane.Orient(ref kinematic.Planes[planes.Count - 1]);
            planes.Add(toolPlane);
        }
        else
            planes.Add(planes[planes.Count - 1]);

        kinematic.Planes = planes.ToArray();
        kinematics.Add(kinematic);
        return kinematics;
    }

    internal override double Payload(int group)
    {
        return Robot.Payload;
    }

    internal override Joint[] GetJoints(int group)
    {
        return Robot.Joints;
    }

    internal override List<List<List<string>>> Code(Program program) => new URScriptPostProcessor(this, program).Code;

    internal override void SaveCode(IProgram program, string folder)
    {
        if (!Directory.Exists(folder))
            throw new DirectoryNotFoundException($" Folder \"{folder}\" not found");

        if (program.Code is null)
            throw new ArgumentNullException(" Program code not generated");

        string file = Path.Combine(folder, $"{program.Name}.URS");
        var joinedCode = string.Join("\r\n", program.Code[0].SelectMany(c => c));
        File.WriteAllText(file, joinedCode);
    }
}
