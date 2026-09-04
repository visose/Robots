using System.Collections;

namespace Robots.Commands;

public class Group(IReadOnlyList<Command> commands) : Command("GroupCommand"), IReadOnlyList<Command>
{
    readonly Command[] _items = [.. commands];

    public Group() : this([]) { }

    public Command this[int index] => _items[index];
    public int Count => _items.Length;
    public IEnumerator<Command> GetEnumerator() => ((IEnumerable<Command>)_items).GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => _items.GetEnumerator();

    internal override IEnumerable<Command> Flatten()
    {
        if (RunBefore)
            throw new InvalidOperationException("RunBefore must be set on commands inside the group.");

        foreach (var command in _items)
        {
            foreach (var item in command.Flatten())
                yield return item;
        }
    }

    public override string ToString() => $"Command (Group with {Count} commands)";
}
