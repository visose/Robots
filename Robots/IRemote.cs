using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public interface IRemote
    {
        List<string> Log { get; }
        string IP { get; set; }
        void Upload(Program program);
        void Pause();
        void Play();
    }
}
