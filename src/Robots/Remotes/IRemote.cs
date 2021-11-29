using System.Collections.Generic;

namespace Robots
{
    public interface IRemote
    {
        List<string> Log { get; }
        string? IP { get; set; }
        void Upload(IProgram program);
        void Pause();
        void Play();
    }
}
