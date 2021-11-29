namespace Robots.Commands
{
    public class Stop : Command
    {
        public Stop() { }

        protected override void Populate()
        {
            _commands.Add(Manufacturers.ABB, (_, __) => "Stop;");
            _commands.Add(Manufacturers.KUKA, (_, __) => "HALT");
            _commands.Add(Manufacturers.UR, (_, __) => "pause program");
            //_commands.Add(Manufacturers.Staubli, (_, __) => "wait(true)");
        }

        public override string ToString() => "Command (Stop)";
    }
}