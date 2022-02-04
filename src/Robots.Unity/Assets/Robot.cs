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
            _program.MeshPoser = new UnityMeshPoser(_program.RobotSystem, _material);
        }

        void Update()
        {
            var time = Mathf.PingPong(Time.time, (float)_program.Duration);
            _program.Animate(time, false);
        }
    }
}