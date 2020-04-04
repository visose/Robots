using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Robots
{
    public interface IToolpath
    {
        IEnumerable<Target> Targets { get; }
        IToolpath ShallowClone(List<Target> targets = null);
    }

    public class SimpleToolpath : IToolpath, IEnumerable<Target>
    {
        public IEnumerable<Target> Targets => _targets;

        protected List<Target> _targets = new List<Target>();

        public SimpleToolpath() { }

        public SimpleToolpath(IEnumerable<IToolpath> toolpaths)
        {
            _targets = new List<Target>();
            _targets.AddRange(toolpaths.SelectMany(t => t.Targets));
        }

        public void Add(Target target)
        {
            _targets.Add(target);
        }

        public IEnumerator<Target> GetEnumerator() => _targets.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => _targets.GetEnumerator();

        public IToolpath ShallowClone(List<Target> targets = null)
        {
            if (targets == null)
                targets = _targets.ToList();

            var clone = MemberwiseClone() as SimpleToolpath;
            clone._targets = targets;
            return clone;
        }
    }
}
