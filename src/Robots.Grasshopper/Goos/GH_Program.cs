﻿using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class GH_Program : GH_Goo<IProgram>
{
    public GH_Program(GH_Program goo) { Value = goo.Value; }
    public GH_Program(IProgram native) { Value = native; }
    public override IGH_Goo Duplicate() => new GH_Program(this);
    public override bool IsValid => true;
    public override string TypeName => "Program";
    public override string TypeDescription => "Program";
    public override string ToString() => Value.ToString();
    public override object? ScriptVariable() => Value;

    public override bool CastFrom(object source)
    {
        switch (source)
        {
            case IProgram program:
                Value = program;
                return true;
            default:
                return false;
        }
    }
}
