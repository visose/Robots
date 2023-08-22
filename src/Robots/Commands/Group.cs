using System.Collections;

namespace Robots.Commands;

public class Group : Command, IList<Command>
{
    readonly List<Command> _commandList = new();

    public Group() { }

    public Group(IEnumerable<Command> commands) : base("GroupCommand")
    {
        _commandList.AddRange(commands);
    }

    // IList implementation
    public Command this[int index]
    {
        get => _commandList[index];
        set => _commandList[index] = value;
    }

    public int IndexOf(Command item) => _commandList.IndexOf(item);
    public void Insert(int index, Command item) => _commandList.Insert(index, item);
    public void RemoveAt(int index) => _commandList.RemoveAt(index);
    public int Count => _commandList.Count;
    public bool IsReadOnly => false;
    public void Add(Command item) => _commandList.Add(item);
    public void Clear() => _commandList.Clear();
    public bool Contains(Command item) => _commandList.Contains(item);
    public void CopyTo(Command[] array, int arrayIndex) => _commandList.CopyTo(array, arrayIndex);
    public bool Remove(Command item) => _commandList.Remove(item);
    public IEnumerator<Command> GetEnumerator() => _commandList.GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => _commandList.GetEnumerator();

    public void AddRange(IEnumerable<Command> source) => _commandList.AddRange(source);

    internal override IEnumerable<Command> Flatten()
    {
        return _commandList.SelectMany(c => c.Flatten());
    }
    public override string ToString() => $"Command (Group with {Count} commands)";
}
