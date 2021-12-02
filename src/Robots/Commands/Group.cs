using System.Collections;
using System.Collections.Generic;

namespace Robots.Commands
{
    public class Group : Command, IList<Command>
    {
        readonly List<Command> _commandList = new();

        public Group()
        {
        }

        public Group(IEnumerable<Command> commands)
        {
            Name = "Group command";
            _commandList.AddRange(commands);
        }

        // IList implementation
        public Command this[int index]
        {
            get { return _commandList[index]; }
            set { _commandList[index] = value; }
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

        public IEnumerable<Command> Flatten()
        {
            var commands = new List<Command>();
            foreach (var command in _commandList)
            {
                if (command is Group group)
                    commands.AddRange(group.Flatten());
                else
                    commands.Add(command);
            }

            return commands;
        }
        public override string ToString() => $"Command (Group with {Count} commands)";
    }
}