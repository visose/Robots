using Rhino.Geometry;
using System;
using System.Collections.Generic;
using System.Linq;
using static Rhino.RhinoMath;
using static Robots.Util;
using static System.Math;

namespace Robots
{
    public class Collision
    {
        Program program;
        RobotSystem robotSystem;
        double linearStep;
        double angularStep;
        IEnumerable<int> first;
        IEnumerable<int> second;
        Mesh environment;
        int environmentPlane;

        public bool HasCollision { get; private set; } = false;
        public Mesh[] Meshes { get; private set; }
        public CellTarget CollisionTarget { get; private set; }

        internal Collision(Program program, IEnumerable<int> first, IEnumerable<int> second, Mesh environment, int environmentPlane, double linearStep, double angularStep)
        {
            this.program = program;
            this.robotSystem = program.RobotSystem;
            this.linearStep = linearStep;
            this.angularStep = angularStep;
            this.first = first;
            this.second = second;
            this.environment = environment;
            this.environmentPlane = environmentPlane;

            Collide();
        }

        void Collide()
        {

            System.Threading.Tasks.Parallel.ForEach(program.Targets, (cellTarget, state) =>
            {
                if (cellTarget.Index == 0) return;
                var prevcellTarget = program.Targets[cellTarget.Index - 1];

                double divisions = 1;

                int groupCount = cellTarget.ProgramTargets.Count;

                for (int group = 0; group < groupCount; group++)
                {
                    var target = cellTarget.ProgramTargets[group];
                    var prevTarget = prevcellTarget.ProgramTargets[group];

                    double distance = prevTarget.WorldPlane.Origin.DistanceTo(target.WorldPlane.Origin);
                    double linearDivisions = Ceiling(distance / linearStep);

                    double maxAngle = target.Kinematics.Joints.Zip(prevTarget.Kinematics.Joints, (x, y) => Abs(x - y)).Max();
                    double angularDivisions = Ceiling(maxAngle / angularStep);

                    double tempDivisions = Max(linearDivisions, angularDivisions);
                    if (tempDivisions > divisions) divisions = tempDivisions;
                }

                //  double step = 1.0 / divisions;

                int j = (cellTarget.Index == 1) ? 0 : 1;

                for (int i = j; i < divisions; i++)
                {
                    double t = (double)i / (double)divisions;
                    var kineTargets = cellTarget.Lerp(prevcellTarget, robotSystem, t, 0.0, 1.0);
                    var kinematics = program.RobotSystem.Kinematics(kineTargets, displayMeshes: true);
                    var meshes = kinematics.SelectMany(x => x.Meshes).ToList();

                    if (this.environment != null)
                    {
                        Mesh currentEnvironment = this.environment.DuplicateMesh();
                        if (this.environmentPlane != -1)
                            currentEnvironment.Transform(Transform.PlaneToPlane(Plane.WorldXY, kinematics.SelectMany(x => x.Planes).ToList()[environmentPlane]));
                        meshes.Add(currentEnvironment);
                    }

                    var setA = first.Select(x => meshes[x]);
                    var setB = second.Select(x => meshes[x]);

                    var meshClash = Rhino.Geometry.Intersect.MeshClash.Search(setA, setB, 1, 1);

                    if (meshClash.Length > 0 && (!HasCollision || CollisionTarget.Index > cellTarget.Index))
                    {
                        HasCollision = true;
                        Meshes = new Mesh[] { meshClash[0].MeshA, meshClash[0].MeshB };
                        this.CollisionTarget = cellTarget;
                        state.Break();
                    }
                }
            });
        }
    }
}
