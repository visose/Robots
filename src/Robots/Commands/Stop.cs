namespace Robots.Commands;

public class Stop : Command
{
    public Stop() { }

    protected override void Populate()
    {
        _commands.Add(Manufacturers.ABB, (_, _) => "Stop;");
        _commands.Add(Manufacturers.KUKA, (_, _) => "HALT");
        _commands.Add(Manufacturers.UR, (_, _) => "pause program");
        _commands.Add(Manufacturers.Doosan, (_, _) => "wait_nudge()");
        //_commands.Add(Manufacturers.Staubli, (_, __) => "wait(true)");
        _commands.Add(Manufacturers.Fanuc, (_, _) => ":ABORT ;");
        _commands.Add(Manufacturers.Igus, (_, _) => "<Stop  Descr=\"\" />");
    }

    public override string ToString() => "Command (Stop)";
}
