using NUnit.Framework;
using Rhino.DocObjects;
using Rhino.FileIO;
using Rhino.Geometry;

namespace Robots.Tests;

public class FileIOTests
{
    [Test]
    public void ListRejectsUnknownElementType()
    {
        _ = Assert.Throws<ArgumentOutOfRangeException>(() => FileIO.List((ElementType)(-1)));
    }

    [Test]
    public void RobotSystemCollisionMeshesFallbackToDisplayMeshes()
    {
        using var doc = CreateRobotMeshDoc(addCollision: false);
        var robot = FileIO.ParseRobotSystem(TestRobots.AbbIrb120Xml, Plane.WorldXY, doc);
        var displayMeshes = robot.DefaultPose.Meshes[0];
        var collisionMeshes = robot.DefaultPose.CollisionMeshes[0];

        Assert.Multiple(() =>
        {
            Assert.That(collisionMeshes, Has.Length.EqualTo(displayMeshes.Length));
            Assert.That(collisionMeshes[0], Is.SameAs(displayMeshes[0]));
            Assert.That(collisionMeshes[6], Is.SameAs(displayMeshes[6]));
        });
    }

    [Test]
    public void RobotSystemLoadsCollisionMeshesFromSiblingLayers()
    {
        using var doc = CreateRobotMeshDoc(addCollision: true);
        var robot = FileIO.ParseRobotSystem(TestRobots.AbbIrb120Xml, Plane.WorldXY, doc);
        var displayMeshes = robot.DefaultPose.Meshes[0];
        var collisionMeshes = robot.DefaultPose.CollisionMeshes[0];

        Assert.Multiple(() =>
        {
            Assert.That(displayMeshes[0].Vertices.Count, Is.EqualTo(3));
            Assert.That(collisionMeshes[0].Vertices.Count, Is.EqualTo(6));
            Assert.That(collisionMeshes[1], Is.SameAs(displayMeshes[1]));
            Assert.That(collisionMeshes[2].Vertices.Count, Is.EqualTo(3));
            Assert.That(collisionMeshes[2], Is.Not.SameAs(displayMeshes[2]));
        });
    }

    [TestCase(false)]
    [TestCase(true)]
    public void ToolUsesCollisionMeshWhenPresent(bool addCollision)
    {
        using var doc = CreateToolMeshDoc(addCollision);
        var tool = FileIO.ParseTool(TestRobots.GripperToolXml, doc);

        Assert.Multiple(() =>
        {
            Assert.That(tool.Mesh.Vertices.Count, Is.EqualTo(3));
            Assert.That(tool.CollisionMesh.Vertices.Count, Is.EqualTo(addCollision ? 6 : 3));
            Assert.That(ReferenceEquals(tool.CollisionMesh, tool.Mesh), Is.EqualTo(!addCollision));
        });
    }

    static File3dm CreateRobotMeshDoc(bool addCollision)
    {
        File3dm doc = new();
        var parentId = AddParentLayer(doc, "RobotArm.ABB.IRB120");

        for (int i = 0; i < 7; i++)
        {
            var layer = AddLayer(doc, i.Text(), parentId);
            AddMesh(doc, layer, TriangleMesh(i));
        }

        if (!addCollision)
            return doc;

        var collisionParentId = AddParentLayer(doc, "RobotArm.ABB.IRB120.Collision");
        var baseCollisionLayer = AddLayer(doc, "0", collisionParentId);
        AddMesh(doc, baseCollisionLayer, TriangleMesh(10));
        AddMesh(doc, baseCollisionLayer, TriangleMesh(20));

        var jointCollisionLayer = AddLayer(doc, "2", collisionParentId);
        AddMesh(doc, jointCollisionLayer, TriangleMesh(30));

        return doc;
    }

    static File3dm CreateToolMeshDoc(bool addCollision)
    {
        File3dm doc = new();
        var displayLayer = AddLayer(doc, "Tool.Gripper");
        AddMesh(doc, displayLayer, TriangleMesh(0));

        if (!addCollision)
            return doc;

        var collisionLayer = AddLayer(doc, "Tool.Gripper.Collision");
        AddMesh(doc, collisionLayer, TriangleMesh(10));
        AddMesh(doc, collisionLayer, TriangleMesh(20));
        return doc;
    }

    static Guid AddParentLayer(File3dm doc, string name)
    {
        _ = AddLayer(doc, name);
        return doc.AllLayers.Last().Id;
    }

    static int AddLayer(File3dm doc, string name, Guid parentId = default)
    {
        Layer layer = new()
        {
            Name = name,
            ParentLayerId = parentId
        };

        doc.AllLayers.Add(layer);
        return doc.AllLayers.Count - 1;
    }

    static void AddMesh(File3dm doc, int layerIndex, Mesh mesh)
    {
        ObjectAttributes attributes = new()
        {
            LayerIndex = layerIndex
        };

        _ = doc.Objects.AddMesh(mesh, attributes);
    }

    static Mesh TriangleMesh(int x)
    {
        Mesh mesh = new();
        _ = mesh.Vertices.Add(x, 0, 0);
        _ = mesh.Vertices.Add(x + 1, 0, 0);
        _ = mesh.Vertices.Add(x, 1, 0);
        _ = mesh.Faces.AddFace(0, 1, 2);
        return mesh;
    }
}
