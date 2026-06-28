using Rhino.Geometry;

namespace Robots;

readonly record struct MechanismBase
{
    public Plane Plane { get; }
    public Mesh Mesh { get; }
    public Mesh CollisionMesh { get; }

    public MechanismBase(Plane plane, Mesh mesh, Mesh? collisionMesh = null)
    {
        Plane = plane;
        Mesh = mesh;
        CollisionMesh = collisionMesh ?? mesh;
    }
}
