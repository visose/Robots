using System.Collections;

namespace Robots.Commands;

public class Group : Command, IReadOnlyList<Command>
{
    readonly Command[] _items;

    public Group(bool runBefore = false) : base(runBefore: runBefore)
    {
        _items = [];
    }

    public Group(IReadOnlyList<Command> commands, bool runBefore = false) : base("GroupCommand", runBefore)
    {
        _items = [.. commands];
    }

    public Command this[int index] => _items[index];
    public int Count => _items.Length;
    public IEnumerator<Command> GetEnumerator() => ((IEnumerable<Command>)_items).GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => _items.GetEnumerator();

    internal override IEnumerable<Command> Flatten()
    {
        return _items.SelectMany(c => c.Flatten());
    }

    public override string ToString() => $"Command (Group with {Count} commands)";
}
