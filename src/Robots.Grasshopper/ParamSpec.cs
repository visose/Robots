namespace Robots.Grasshopper;

sealed record ParamSpec(
    string Name,
    string NickName,
    string Description,
    bool Optional,
    Func<IGH_Param> CreateParam)
{
    public string Menu(string suffix) => $"{Name} {suffix}";

    public IGH_Param Create()
    {
        var param = CreateParam();
        ApplyToParam(param);
        return param;
    }

    public void ApplyToParam(IGH_Param param)
    {
        param.Name = Name;
        param.NickName = NickName;
        param.Description = Description;
        param.Optional = Optional;
    }

    public static ParamSpec New<TParam>(string name, string nickname, string description, bool optional)
        where TParam : IGH_Param, new() =>
        new(name, nickname, description, optional, () => new TParam());

    public static int IndexOf(IReadOnlyList<ParamSpec> specs, string name)
    {
        for (int i = 0; i < specs.Count; i++)
        {
            if (specs[i].Name == name)
                return i;
        }

        return -1;
    }
}
