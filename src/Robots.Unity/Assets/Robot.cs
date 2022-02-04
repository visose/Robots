using System;
using UnityEngine;

namespace Robots.Unity
{
    public class Robot : MonoBehaviour
    {
        [SerializeField]
        Material? _material;

        Program _program = default!;

        void Start()
        {
            if (_material == null)
                throw new ArgumentNullException(nameof(_material));

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