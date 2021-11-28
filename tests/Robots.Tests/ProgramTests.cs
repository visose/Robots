using System;
using System.Linq;
using System.Security.Cryptography;
using Rhino.Geometry;
using NUnit.Framework;

namespace Robots.Tests
{
    public class ProgramTests
    {
        readonly Program _program;

        public ProgramTests()
        {
            var xml = "  <RobotCell name=\"Bartlett-IRB120\" manufacturer=\"ABB\">    <Mechanisms>      <RobotArm model=\"IRB120\" manufacturer=\"ABB\" payload=\"3\">        <Base x=\"0.000\" y=\"0.000\" z=\"0.000\" q1=\"1.000\" q2=\"0.000\" q3=\"0.000\" q4=\"0.000\"/>        <Joints>          <Revolute number=\"1\" a =\"0\" d =\"290\" minrange = \"-165\" maxrange =\"165\" maxspeed =\"250\"/>          <Revolute number=\"2\" a =\"270\" d =\"0\" minrange = \"-110\" maxrange =\"110\" maxspeed =\"250\"/>          <Revolute number=\"3\" a =\"70\" d =\"0\" minrange = \"-110\" maxrange =\"70\" maxspeed =\"250\"/>          <Revolute number=\"4\" a =\"0\" d =\"302\" minrange = \"-160\" maxrange =\"160\" maxspeed =\"320\"/>          <Revolute number=\"5\" a =\"0\" d =\"0\" minrange = \"-120\" maxrange =\"120\" maxspeed =\"320\"/>          <Revolute number=\"6\" a =\"0\" d =\"72\" minrange = \"-400\" maxrange =\"400\" maxspeed =\"420\"/>        </Joints>      </RobotArm>    </Mechanisms>    <IO>      <DO names=\"DO10_1,DO10_2,DO10_3,DO10_4,DO10_5,DO10_6,DO10_7,DO10_8,DO10_9,DO10_10,DO10_11,DO10_12,DO10_13,DO10_14,DO10_15,DO10_16\"/>      <DI names=\"DI10_1,DI10_2,DI10_3,DI10_4,DI10_5,DI10_6,DI10_7,DI10_8,DI10_9,DI10_10,DI10_11,DI10_12,DI10_13,DI10_14,DI10_15,DO10_16\"/>    </IO>  </RobotCell>";
            var robot = RobotSystem.Parse(xml, Plane.WorldXY);

            var planeA = Plane.WorldYZ;
            var planeB = Plane.WorldYZ;
            planeA.Origin = new Point3d(300, 200, 610);
            planeB.Origin = new Point3d(300, -200, 610);
            var speed = new Speed(300);
            var targetA = new CartesianTarget(planeA, RobotConfigurations.Wrist, Motions.Joint);
            var targetB = new CartesianTarget(planeB, null, Motions.Linear, speed: speed);
            var toolpath = new SimpleToolpath() { targetA, targetB };

            _program = new Program("TestProgram", robot, new[] { toolpath });
        }

        [Test]
        public void BasicProgramCorrectDuration()
        {
            const double expected = 1.6432545251573487;
            Assert.AreEqual(_program.Duration, expected);
        }

        [Test]
        public void BasicProgramCorrectJoints()
        {
            double[] expected =
            {
                -0.72007069377409672,
                1.5806369662963811,
                -0.075569321979534809,
                -1.4960590345094886,
                0.72252891341688164,
                3.042104596978858
            };

            var actual = _program.Targets[1].Joints;

            for (int i = 0; i < 6; i++)
            {
                Assert.AreEqual(actual[i], expected[i]);
            }
        }

        [Test]
        public void BasicProgramCorrectPlanes()
        {
            const string expected = "58bbdd38df197a53164a9398d5b3c627";

            var values = _program.Targets[1].Planes
                .SelectMany(p => new[] { (Vector3d)p.Origin, p.XAxis, p.YAxis })
                .SelectMany(v => new[] { v.X, v.Y, v.Z });

            var actual = GetHash(values.ToArray());
            Assert.AreEqual(actual, expected);
        }

        static string GetHash(double[] values)
        {
            var bytes = new byte[values.Length * sizeof(double)];
            Buffer.BlockCopy(values, 0, bytes, 0, bytes.Length);
            var hash = MD5.HashData(bytes);                 
            return string.Concat(hash.Select(b => b.ToString("x2")));            
        }
    }
}