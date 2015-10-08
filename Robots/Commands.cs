using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Rhino.Geometry;
using System.Collections;

namespace Robots.Commands
{
    public interface ICommand
    {
        string CodeABB();
        string CodeKUKA();
        string CodeUR();
    }

    public class Custom : ICommand
    {
        string name;
        string ABB;
        string KUKA;
        string UR;

        public Custom(string name = "Custom command", string ABB = null, string KUKA = null, string UR = null)
        {
            this.name = name;
            this.ABB = ABB;
            this.KUKA = KUKA;
            this.UR = UR;
        }

        public string CodeABB() => this.ABB;
        public string CodeKUKA() => this.KUKA;
        public string CodeUR() => this.UR;

        public override string ToString() => name;
    }

    public class List : IList<ICommand>, ICommand
    {
        private readonly List<ICommand> list = new List<ICommand>();

        public string CodeABB() => string.Join("\r\n", list.Select(command => command.CodeABB()));
        public string CodeKUKA() => string.Join("\r\n", list.Select(command => command.CodeKUKA()));
        public string CodeUR() => string.Join("\r\n", list.Select(command => command.CodeUR()));

        public IEnumerator<ICommand> GetEnumerator() => list.GetEnumerator();
        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

        public void Add(ICommand item) { list.Add(item); }
        public void AddRange(IEnumerable<ICommand> items) { list.AddRange(items); }
        public void Clear() { list.Clear(); }
        public bool Contains(ICommand item) => list.Contains(item);
        public void CopyTo(ICommand[] array, int arrayIndex) { list.CopyTo(array, arrayIndex); }
        public bool Remove(ICommand item) => list.Remove(item);
        public int Count => list.Count;
        public bool IsReadOnly => false;

        public int IndexOf(ICommand item) => list.IndexOf(item);
        public void Insert(int index, ICommand item) { list.Insert(index, item); }
        public void RemoveAt(int index) { list.RemoveAt(index); }
        public ICommand this[int index]
        {
            get { return list[index]; }
            set { list[index] = value; }
        }
    }

    public class Wait : ICommand
    {
        public string CodeABB() => null;
        public string CodeKUKA() => null;
        public string CodeUR() => null;
    }
}
