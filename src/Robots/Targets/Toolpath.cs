using System.Collections;

namespace Robots;

public interface IToolpath
{
    IEnumerable<Target> Targets { get; }
    IToolpath ShallowClone(List<Target>? targets = null);
}

public class SimpleToolpath : IToolpath, IEnumerable<Target>
{
    protected List<Target> _targets = [];
    public IEnumerable<Target> Targets => _targets;

    public SimpleToolpath() { }

    public SimpleToolpath(IEnumerable<IToolpath> toolpaths)
    {
        _targets = [.. toolpaths.SelectMany(t => t.Targets)];
    }

    public void Add(Target target)
    {
        _targets.Add(target);
    }

    public IEnumerator<Target> GetEnumerator() => _targets.GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => _targets.GetEnumerator();

    public IToolpath ShallowClone(List<Target>? targets = null)
    {
        targets ??= [.. _targets];

        var clone = (SimpleToolpath)MemberwiseClone();
        clone._targets = targets;
        return clone;
    }
}
