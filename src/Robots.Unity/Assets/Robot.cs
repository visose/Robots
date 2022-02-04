using UnityEngine;

namespace Robots.Unity
{
    public class Robot : MonoBehaviour
    {
        [SerializeField]
        Material _material;

        Program _program;

        void Start()
        {
            _program = TestProgram.Create();
            var cell = (RobotCell)_program.RobotSystem;
            var poser = new UnityMeshPoser(cell, _material);
            _program.MeshPoser = poser;
        }

        void Update()
        {
            var time = Mathf.PingPong(Time.time, (float)_program.Duration);
            _program.Animate(time, false);
        }
    }
}