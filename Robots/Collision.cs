#define RHINOCOMMON

using Rhino.Geometry;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using static System.Math;

namespace Robots
{
    public class Collision
    {
        readonly Program program;
        readonly RobotSystem robotSystem;
        readonly double linearStep;
        readonly double angularStep;
        readonly IEnumerable<int> first;
        readonly IEnumerable<int> second;
        readonly Mesh environment;
        readonly int environmentPlane;
        readonly bool _onlyOne;
        readonly int _oneFirst;
        readonly int _oneSecond;

        public bool HasCollision { get; private set; } = false;
        public Mesh[] Meshes { get; private set; }
        public CellTarget CollisionTarget { get; private set; }

        internal Collision(Program program, IEnumerable<int> first, IEnumerable<int> second, Mesh environment, int environmentPlane, double linearStep, double angularStep)
        {
#if RHINOCOMMON
            this.program = program;
            this.robotSystem = program.RobotSystem;
            this.linearStep = linearStep;
            this.angularStep = angularStep;
            this.first = first;
            this.second = second;
            this.environment = environment;
            this.environmentPlane = environmentPlane;

            if (first.Count() == 1 && second.Count() == 1)
            {
                _onlyOne = true;
                _oneFirst = first.First();
                _oneSecond = second.First();
            }

            Collide();
#else
            throw new NotImplementedException(" Collisions have to be reimplemented.");
#endif
        }

        void Collide()
        {
            Parallel.ForEach(program.Targets, (cellTarget, state) =>
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

                var meshes = new List<Mesh>();

                int j = (cellTarget.Index == 1) ? 0 : 1;

                for (int i = j; i < divisions; i++)
                {
                    double t = (double)i / (double)divisions;
                    var kineTargets = cellTarget.Lerp(prevcellTarget, robotSystem, t, 0.0, 1.0);
                    var kinematics = program.RobotSystem.Kinematics(kineTargets);

                    meshes.Clear();

                    // TODO: Meshes not a property of KinematicSolution anymore
                    // meshes.AddRange(kinematics.SelectMany(x => x.Meshes)); 
                    var tools = cellTarget.ProgramTargets.Select(p => p.Target.Tool.Mesh).ToList();
                    var robotMeshes = PoseMeshes(program.RobotSystem, kinematics, tools);
                    meshes.AddRange(robotMeshes);

                    if (this.environment != null)
                    {
                        if (this.environmentPlane != -1)
                        {
                            Mesh currentEnvironment = this.environment.DuplicateMesh();
                            currentEnvironment.Transform(Transform.PlaneToPlane(Plane.WorldXY, kinematics.SelectMany(x => x.Planes).ToList()[environmentPlane]));
                            meshes.Add(currentEnvironment);
                        }
                        else
                        {
                            meshes.Add(this.environment);
                        }
                    }

                    if (_onlyOne)
                    {
                        var meshA = meshes[_oneFirst];
                        var meshB = meshes[_oneSecond];

                        var meshClash = Rhino.Geometry.Intersect.Intersection.MeshMeshFast(meshA, meshB);

                        if (meshClash.Length > 0 && (!HasCollision || CollisionTarget.Index > cellTarget.Index))
                        {
                            HasCollision = true;
                            Meshes = new Mesh[] { meshA, meshB };
                            this.CollisionTarget = cellTarget;
                            state.Break();
                        }
                    }
                    else
                    {
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
                }
            });
        }

        public static List<Mesh> PoseMeshes(RobotSystem robot, List<KinematicSolution> solutions, List<Mesh> tools)
        {
            if (robot is RobotCell cell)
            {
                var meshes = solutions.SelectMany((_, i) => PoseMeshes(cell.MechanicalGroups[i], solutions[i].Planes, tools[i])).ToList();
                return meshes;
            }
            else
            {
                var ur = robot as RobotCellUR;
                var meshes = PoseMeshesRobot(ur.Robot, solutions[0].Planes, tools[0]);
                return meshes;
            }
        }

        static List<Mesh> PoseMeshes(MechanicalGroup group, IList<Plane> planes, Mesh tool)
        {
            planes = planes.ToList();
            var count = planes.Count - 1;
            planes.RemoveAt(count);
            planes.Add(planes[count - 1]);

            var outMeshes = group.DefaultMeshes.Select(m => m.DuplicateMesh()).Append(tool.DuplicateMesh()).ToList();

            for (int i = 0; i < group.DefaultPlanes.Count; i++)
            {
                var s = Transform.PlaneToPlane(group.DefaultPlanes[i], planes[i]);
                outMeshes[i].Transform(s);
            }

            return outMeshes;
        }

        static List<Mesh> PoseMeshesRobot(RobotArm arm, IList<Plane> planes, Mesh tool)
        {
            planes = planes.ToList();
            var count = planes.Count - 1;
            planes.RemoveAt(count);
            planes.Add(planes[count - 1]);

            var defaultPlanes = arm.Joints.Select(m => m.Plane).Prepend(arm.BasePlane).Append(Plane.WorldXY).ToList();
            var defaultMeshes = arm.Joints.Select(m => m.Mesh).Prepend(arm.BaseMesh).Append(tool);
            var outMeshes = defaultMeshes.Select(m => m.DuplicateMesh()).ToList();

            for (int i = 0; i < defaultPlanes.Count; i++)
            {
                var s = Transform.PlaneToPlane(defaultPlanes[i], planes[i]);
                outMeshes[i].Transform(s);
            }

            return outMeshes;
        }
    }
}
