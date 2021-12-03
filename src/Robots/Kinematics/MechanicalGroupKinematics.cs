using Rhino.Geometry;

namespace Robots;

class MechanicalGroupKinematics : KinematicSolution
{
    internal MechanicalGroupKinematics(MechanicalGroup group, Target target, double[]? prevJoints, Plane? coupledPlane, Plane? basePlane)
    {
        var jointCount = group.Joints.Count;
        Joints = new double[jointCount];
        var planes = new List<Plane>();
        var errors = new List<string>();

        Plane? robotBase = basePlane;

        target = target.ShallowClone();
        Mechanism? coupledMech = null;

        if (target.Frame.CoupledMechanism != -1 && target.Frame.CoupledMechanicalGroup == group.Index)
        {
            coupledMech = group.Externals[target.Frame.CoupledMechanism];
        }

        // Externals
        foreach (var external in group.Externals)
        {
            var externalPrevJoints = prevJoints?.Subset(external.Joints.Select(x => x.Number).ToArray());
            var externalKinematics = external.Kinematics(target, externalPrevJoints, basePlane);

            for (int i = 0; i < external.Joints.Length; i++)
                Joints[external.Joints[i].Number] = externalKinematics.Joints[i];

            planes.AddRange(externalKinematics.Planes);
            errors.AddRange(externalKinematics.Errors);

            if (external == coupledMech)
                coupledPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];

            if (external.MovesRobot)
            {
                //Plane plane = (robotBase is not null) ? robotBase.Value : Plane.WorldXY;
                Plane externalPlane = externalKinematics.Planes[externalKinematics.Planes.Length - 1];

                //plane.Transform(externalPlane.ToTransform());
                //robotBase = plane;
                robotBase = externalPlane;
            }
        }

        // Coupling
        if (coupledPlane is not null)
        {
            var coupledFrame = target.Frame.ShallowClone();
            var plane = coupledFrame.Plane;
            plane.Transform(Transform.PlaneToPlane(Plane.WorldXY, (Plane)coupledPlane));
            coupledFrame.Plane = plane;
            target.Frame = coupledFrame;
        }

        // Robot
        var robot = group.Robot;

        if (robot is not null)
        {
            var robotPrevJoints = prevJoints?.Subset(robot.Joints.Select(x => x.Number).ToArray());
            var robotKinematics = robot.Kinematics(target, robotPrevJoints, robotBase);

            for (int j = 0; j < robot.Joints.Length; j++)
                Joints[robot.Joints[j].Number] = robotKinematics.Joints[j];

            planes.AddRange(robotKinematics.Planes);
            Configuration = robotKinematics.Configuration;

            if (robotKinematics.Errors.Count > 0)
            {
                errors.AddRange(robotKinematics.Errors);
            }
        }

        // Tool
        Plane toolPlane = target.Tool.Tcp;
        toolPlane.Transform(planes[planes.Count - 1].ToTransform());
        planes.Add(toolPlane);

        Planes = planes.ToArray();

        if (errors.Count > 0)
        {
            Errors.AddRange(errors);
        }
    }
}
