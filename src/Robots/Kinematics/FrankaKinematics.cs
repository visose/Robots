using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots;

class FrankaKinematics : RobotKinematics
{
    public FrankaKinematics(RobotArm robot, Target target, double[]? prevJoints, Plane? basePlane)
        : base(robot, target, prevJoints, basePlane) { }

    /// <summary>
    /// Code adapted from https://github.com/ffall007/franka_analytical_ik
    /// </summary>
    protected override double[] InverseKinematics(Transform transform, RobotConfigurations configuration, double[] external, double[]? prevJoints, out List<string> errors)
    {
        double M_PI_4 = 0;

        var joints = _mechanism.Joints;
        var a = joints.Map(j => j.A);
        var d = joints.Map(j => j.D);
        var q_min = joints.Map(j => j.Range.Min);
        var q_max = joints.Map(j => j.Range.Max);

        errors = new List<string>();
        bool isUnreachable = false;

        double[] q_actual_array;
        if (prevJoints is null)
        {
            q_actual_array = new double[7];

            for (int i = 0; i < 7; i++)
                q_actual_array[i] = (q_min[i] + q_max[i]) * 0.5;
        }
        else
        {
            q_actual_array = prevJoints;
        }

        double q7 =
             external.Length > 0
            ? external[0]
            : q_actual_array[6];

        var zeroes = new double[] { 0, 0, 0, 0, 0, 0, 0 };

        var q_all = new double[4][]
        {
            new double[] {double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN },
            new double[] {double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN },
            new double[] {double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN },
            new double[] {double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN, double.NaN }
        };

        double d1 = d[0];
        double d3 = d[2];
        double d5 = d[4];
        double d7e = d[6];
        double a4 = a[3];
        double a7 = a[6];

        double LL24 = a4 * a4 + d3 * d3;
        double LL46 = a4 * a4 + d5 * d5;
        double L24 = Sqrt(LL24);
        double L46 = Sqrt(LL46);

        double thetaH46 = Atan(d5 / a4);
        double theta342 = Atan(d3 / a4);
        double theta46H = Atan2(1, d5 / a4);

        for (int i = 0; i < 4; i++)
            q_all[i][6] = q7;

        // compute p_6
        Vector3d z_EE = transform.GetColumn3d(2);
        Vector3d p_EE = transform.GetColumn3d(3);
        Vector3d p_7 = p_EE - d7e * z_EE;

        Vector3d x_EE_6 = new(Cos(q7 - M_PI_4), -Sin(q7 - M_PI_4), 0.0);

        var x_6 = transform * x_EE_6;
        x_6.Normalize();
        Vector3d p_6 = p_7 - a7 * x_6;

        // compute q4
        Vector3d p_2 = new(0.0, 0.0, d1);
        Vector3d V26 = p_6 - p_2;

        double LL26 = V26[0] * V26[0] + V26[1] * V26[1] + V26[2] * V26[2];
        double L26 = Sqrt(LL26);

        //if (L24 + L46 < L26 || L24 + L26 < L46 || L26 + L46 < L24)

        double theta246 = Acos((LL24 + LL46 - LL26) / 2.0 / L24 / L46);

        if (double.IsNaN(theta246))
        {
            theta246 = Math.PI;
            isUnreachable = true;
        }

        double q4 = theta246 + thetaH46 + theta342 - 2.0 * PI;

        for (int i = 0; i < 4; i++)
            q_all[i][3] = q4;

        // compute q6
        double theta462 = Acos((LL26 + LL46 - LL24) / 2.0 / L26 / L46);

        if (double.IsNaN(theta462))
        {
            theta462 = 0;
            isUnreachable = true;
        }

        double theta26H = theta46H + theta462;
        double D26 = -L26 * Cos(theta26H);

        var Z_6 = Vector3d.CrossProduct(z_EE, x_6);
        Z_6.Normalize();
        var Y_6 = Vector3d.CrossProduct(Z_6, x_6);
        Y_6.Normalize();

        Transform R_6 = default;
        for (int i = 0; i < 3; i++)
        {
            R_6[i, 0] = x_6[i];
            R_6[i, 1] = Y_6[i];
            R_6[i, 2] = Z_6[i];
        }

        Vector3d V_6_62 = R_6.Transpose() * (-V26);

        double Phi6 = Atan2(V_6_62[1], V_6_62[0]);
        double Theta6 = Asin(D26 / Sqrt(V_6_62[0] * V_6_62[0] + V_6_62[1] * V_6_62[1]));

        if (double.IsNaN(Theta6))
        {
            Theta6 = 0;

            if (!isUnreachable)
                errors.Add("Target not reachable.");
        }

        var q6 = new double[2];
        q6[0] = PI - Theta6 - Phi6;
        q6[1] = Theta6 - Phi6;

        for (int i = 0; i < 2; i++)
        {
            if (q6[i] <= q_min[5])
                q6[i] += 2.0 * PI;
            else if (q6[i] >= q_max[5])
                q6[i] -= 2.0 * PI;

            q_all[2 * i][5] = q6[i];
            q_all[2 * i + 1][5] = q6[i];
        }

        //if (double.IsNaN(q_all[0][5]) && double.IsNaN(q_all[2][5]))

        // compute q1 & q2
        double thetaP26 = 3.0 * HalfPI - theta462 - theta246 - theta342;
        double thetaP = PI - thetaP26 - theta26H;
        double LP6 = L26 * Sin(thetaP26) / Sin(thetaP);

        var z_5_all = new Vector3d[4];
        var V2P_all = new Vector3d[4];

        for (int i = 0; i < 2; i++)
        {
            Vector3d z_6_5 = new(Sin(q6[i]), Cos(q6[i]), 0.0);
            Vector3d z_5 = R_6 * z_6_5;
            Vector3d V2P = p_6 - LP6 * z_5 - p_2;

            z_5_all[2 * i] = z_5;
            z_5_all[2 * i + 1] = z_5;
            V2P_all[2 * i] = V2P;
            V2P_all[2 * i + 1] = V2P;

            double L2P = V2P.Length;

            if (Abs(V2P[2] / L2P) > 0.999)
            {
                q_all[2 * i][0] = q_actual_array[0];
                q_all[2 * i][1] = 0.0;
                q_all[2 * i + 1][0] = q_actual_array[0];
                q_all[2 * i + 1][1] = 0.0;
            }
            else
            {
                q_all[2 * i][0] = Atan2(V2P[1], V2P[0]);
                q_all[2 * i][1] = Acos(V2P[2] / L2P);
                if (q_all[2 * i][0] < 0)
                    q_all[2 * i + 1][0] = q_all[2 * i][0] + PI;
                else
                    q_all[2 * i + 1][0] = q_all[2 * i][0] - PI;
                q_all[2 * i + 1][1] = -q_all[2 * i][1];
            }
        }

        for (int i = 0; i < 4; i++)
        {
            // compute q3
            Vector3d z_3 = V2P_all[i];
            z_3.Normalize();
            var Y_3 = Vector3d.CrossProduct(-V26, V2P_all[i]);
            Vector3d y_3 = Y_3;
            y_3.Normalize();
            var x_3 = Vector3d.CrossProduct(y_3, z_3);

            double c1 = Cos(q_all[i][0]);
            double s1 = Sin(q_all[i][0]);

            Transform R_1 = default;
            R_1.SetRotation(
                c1, -s1, 0.0,
                s1, c1, 0.0,
                0.0, 0.0, 1.0
                );

            double c2 = Cos(q_all[i][1]);
            double s2 = Sin(q_all[i][1]);

            Transform R_1_2 = default;
            R_1_2.SetRotation(
                c2, -s2, 0.0,
                0.0, 0.0, 1.0,
                -s2, -c2, 0.0
                );

            var R_2 = R_1 * R_1_2;

            Vector3d x_2_3 = R_2.Transpose() * x_3;
            q_all[i][2] = Atan2(x_2_3[2], x_2_3[0]);

            // compute q5
            Vector3d VH4 = p_2 + d3 * z_3 + a4 * x_3 - p_6 + d5 * z_5_all[i];

            double c6 = Cos(q_all[i][5]);
            double s6 = Sin(q_all[i][5]);

            Transform R_5_6 = default;
            R_5_6.SetRotation(
                c6, -s6, 0.0,
                0.0, 0.0, -1.0,
                s6, c6, 0.0
                );

            Transform R_5 = R_6 * R_5_6.Transpose();
            var V_5_H4 = R_5.Transpose() * VH4;
            q_all[i][4] = -Atan2(V_5_H4[1], V_5_H4[0]);
        }

        if (isUnreachable)
            errors.Add("Target out of reach.");

        if (configuration.HasFlag(RobotConfigurations.Wrist))
            configuration &= ~RobotConfigurations.Wrist;

        return q_all[(int)configuration];
    }

    protected override Transform[] ForwardKinematics(double[] joints)
    {
        double[] α = new[] { 0, -HalfPI, HalfPI, HalfPI, -HalfPI, HalfPI, HalfPI };
        var t = ModifiedDH(joints, α);
        return t;
    }
}
