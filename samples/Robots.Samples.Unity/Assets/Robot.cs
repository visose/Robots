using System;
using System.Threading.Tasks;
using UnityEngine;

namespace Robots.Samples.Unity
{
    public class Robot : MonoBehaviour
    {
        [SerializeField]
        Material? _material;

        Program? _program;

        async void Start()
        {
            try
            {
                _program = TestProgram.Create();
            }
            catch (ArgumentException e)
            {
                if (!e.Message.Contains("not found"))
                    throw;

                Debug.Log("Bartlett robot library not found, installing...");
                await DownloadLibraryAsync();
                _program = TestProgram.Create();
            }

            if (_material == null)
                throw new ArgumentNullException(nameof(_material));

            _program.MeshPoser = new UnityMeshPoser(_program.RobotSystem, _material);
        }

        async Task DownloadLibraryAsync()
        {
            var online = new OnlineLibrary();
            await online.UpdateLibraryAsync();
            var bartlett = online.Libraries["Bartlett"];
            await online.DownloadLibraryAsync(bartlett);
        }

        void Update()
        {
            if (_program is null)
                return;

            var time = Mathf.PingPong(Time.time, (float)_program.Duration);
            _program.Animate(time, false);
        }
    }
}