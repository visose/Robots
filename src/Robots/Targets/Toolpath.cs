using System.Collections;

namespace Robots;

public interface IToolpath
{
    IReadOnlyList<Target> Targets { get; }
}

public class SimpleToolpath : IToolpath, IReadOnlyList<Target>
{
    readonly Target[] _targets;

    public IReadOnlyList<Target> Targets => _targets;
    public int Count => _targets.Length;
    public Target this[int index] => _targets[index];

    public SimpleToolpath()
    {
        _targets = [];
    }

    public SimpleToolpath(params Target[] targets)
    {
        _targets = [.. targets];
    }

    public SimpleToolpath(IReadOnlyList<Target> targets)
    {
        _targets = [.. targets];
    }

    public SimpleToolpath(IReadOnlyList<IToolpath> toolpaths)
    {
        int count = 0;

        for (int i = 0; i < toolpaths.Count; i++)
            count += toolpaths[i].Targets.Count;

        _targets = new Target[count];
        int index = 0;

        for (int i = 0; i < toolpaths.Count; i++)
        {
            var targets = toolpaths[i].Targets;

            for (int j = 0; j < targets.Count; j++)
                _targets[index++] = targets[j];
        }
    }

    public IEnumerator<Target> GetEnumerator() => ((IEnumerable<Target>)_targets).GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => _targets.GetEnumerator();
}
