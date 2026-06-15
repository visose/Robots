using System.Collections;

namespace Robots;

public interface IToolpath
{
    IReadOnlyList<Target> Targets { get; }
    IToolpath ShallowClone(IReadOnlyList<Target>? targets = null);
}

public class SimpleToolpath : IToolpath, IEnumerable<Target>
{
    protected List<Target> _targets = [];
    public IReadOnlyList<Target> Targets => _targets;

    public SimpleToolpath() { }

    public SimpleToolpath(IReadOnlyList<IToolpath> toolpaths)
    {
        int count = 0;

        for (int i = 0; i < toolpaths.Count; i++)
            count += toolpaths[i].Targets.Count;

        _targets = new List<Target>(count);

        for (int i = 0; i < toolpaths.Count; i++)
        {
            var targets = toolpaths[i].Targets;

            for (int j = 0; j < targets.Count; j++)
                _targets.Add(targets[j]);
        }
    }

    public void Add(Target target)
    {
        _targets.Add(target);
    }

    public IEnumerator<Target> GetEnumerator() => _targets.GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => _targets.GetEnumerator();

    public IToolpath ShallowClone(IReadOnlyList<Target>? targets = null)
    {
        var clone = (SimpleToolpath)MemberwiseClone();
        clone._targets = targets is null ? [.. _targets] : [.. targets];
        return clone;
    }
}
