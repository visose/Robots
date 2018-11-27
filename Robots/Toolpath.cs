using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Robots
{
    public interface IToolpath
    {
        IEnumerable<Target> Targets { get; }
    }

    public class SimpleToolpath : IToolpath
    {
        public IEnumerable<Target> Targets => _targets;

        List<Target> _targets;

        public SimpleToolpath()
        {
            _targets = new List<Target>();
        }

        public SimpleToolpath(IEnumerable<IToolpath> toolpaths)
        {
            _targets = new List<Target>();
            _targets.AddRange(toolpaths.SelectMany(t => t.Targets));
        }
    }
}
