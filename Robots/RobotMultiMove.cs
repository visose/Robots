using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Rhino.Geometry;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    public class RobotMultiMove : Machine
    {
        public Robot RobotA { get; }
        public Robot RobotB { get; }

        internal RobotMultiMove(string name, Manufacturers manufacturer, List<Mechanism> mechanisms, IO io, Plane basePlane, Mesh environment) : base(name, manufacturer, mechanisms, io, basePlane, environment)
        {
            var robots = mechanisms.OfType<Robot>().ToList();
            this.RobotA = robots[0];
            this.RobotB = robots[1];
        }

        public override KinematicSolution Kinematics(Target target, double[] prevJoints = null, bool displayMeshes = false, Plane? basePlane = null) => new RobotMultiMoveKinematics(this, target, prevJoints, displayMeshes);

        internal override void SaveCode(Program program, string folder) { }
        internal override List<List<string>> Code(Program program) => null;


        class RobotMultiMoveKinematics : KinematicSolution
        {
            internal RobotMultiMoveKinematics(RobotMultiMove cell, Target target, double[] prevJoints, bool displayMeshes)
            {
                Joints = new double[12];
                Planes = new Plane[16];
                if (displayMeshes) Meshes = new Mesh[16];

                KinematicSolution robotAKinematics = null;
                KinematicSolution robotBKinematics = null;

                var robotATarget = target.ShallowClone();
                var robotBTarget = target.SubTargets[0].ShallowClone();
                Plane robotBase = cell.RobotA.BasePlane;

                {
                    var robotAPrevJoints = prevJoints.RangeSubset(0, 6);
                    robotAKinematics = cell.RobotA.Kinematics(robotATarget, robotAPrevJoints, displayMeshes);

                    //  var newFrame = robotAKinematics.Planes[robotAKinematics.Planes.Length - 1];

                    if (robotBTarget is CartesianTarget && robotBTarget.Frame != null && robotBTarget.Frame.IsCoupled)
                    {
                        var plane = (robotBTarget as CartesianTarget).Plane;
                        plane.Transform(Transform.PlaneToPlane(robotAKinematics.Planes[robotAKinematics.Planes.Length - 1], robotBTarget.Frame.Plane));
                        (robotBTarget as CartesianTarget).Plane = plane;
                    }
                }

                var robotBPrevJoints = prevJoints.RangeSubset(6, 6);
                robotBKinematics = cell.RobotB.Kinematics(robotBTarget, robotBPrevJoints, displayMeshes);

                // Joints
                {
                    for (int i = 0; i < 6; i++)
                    {
                        Joints[i] = robotAKinematics.Joints[i];
                        Joints[i + 6] = robotBKinematics.Joints[i];
                    }
                }

                // Planes
                {

                    for (int i = 0; i < 8; i++)
                    {
                        Planes[i] = robotAKinematics.Planes[i];
                        Planes[i + 8] = robotBKinematics.Planes[i];
                    }
                }

                // Meshes
                if (displayMeshes)
                {
                    for (int i = 0; i < 8; i++)
                    {
                        Meshes[i] = robotAKinematics.Meshes[i];
                        Meshes[i + 8] = robotBKinematics.Meshes[i];
                    }
                }

                // Errors
                if (robotAKinematics != null) Errors.AddRange(robotAKinematics.Errors);
                if (robotBKinematics != null) Errors.AddRange(robotBKinematics.Errors);

                // Configuration
                Configuration = robotAKinematics.Configuration;
            }
        }
    }
}
