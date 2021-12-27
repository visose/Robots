using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using Rhino.Geometry;

namespace Robots.Standalone;

class HelixMeshPoser : IMeshPoser
{
    readonly List<Plane> _defaultPlanes;
    readonly ObservableElement3DCollection _robotModels;

    public HelixMeshPoser(RobotCell cell, PBRMaterial material, ObservableElement3DCollection robotModels)
    {
        var group = cell.MechanicalGroups.First();
        var meshes = group.DefaultMeshes;
        _defaultPlanes = group.DefaultPlanes;
        _robotModels = robotModels;

        foreach (var joint in meshes)
        {
            var model = new MeshGeometryModel3D
            {
                Geometry = joint.ToWPF(),
                Material = material,
                Transform = Transform3D.Identity,
                IsThrowingShadow = true
            };

            robotModels.Add(model);
        }
    }

    public void Pose(List<KinematicSolution> solutions, Tool[] tools)
    {
        var planes = solutions[0].Planes;

        for (int i = 0; i < planes.Length - 1; i++)
        {
            var s = Transform.PlaneToPlane(_defaultPlanes[i], planes[i]);
            var matrix = new SharpDX.Matrix((float)s.M00, (float)s.M10, (float)s.M20, (float)s.M30, (float)s.M01, (float)s.M11, (float)s.M21, (float)s.M31, (float)s.M02, (float)s.M12, (float)s.M22, (float)s.M32, (float)s.M03, (float)s.M13, (float)s.M23, (float)s.M33);
            var model = _robotModels[i];
            model.SceneNode.ModelMatrix = matrix;
        }
    }
}