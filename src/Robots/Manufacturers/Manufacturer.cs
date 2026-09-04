using static Robots.Util;

namespace Robots;

public enum Manufacturers { ABB, KUKA, UR, Staubli, FrankaEmika, Doosan, Fanuc, Igus, Jaka, All };

abstract class ManufacturerDefinition
{
    public abstract Manufacturers Id { get; }

    public abstract RobotArm CreateRobot(
        string model,
        double payload,
        MechanismBase mechanismBase,
        Joint[] joints);

    public abstract RobotSystem CreateSystem(
        SystemAttributes attributes,
        List<MechanicalGroup> mechanicalGroups);

    public virtual int? ControllerIOStartIndex => null;

    protected MechanicalGroup GetSingleGroup<T>(List<MechanicalGroup> mechanicalGroups)
        where T : RobotArm
    {
        if (mechanicalGroups.Count != 1)
            throw new ArgumentException($"{Id} robot systems must contain exactly one mechanical group.");

        var group = mechanicalGroups[0];

        if (group.Robot is not T)
            throw new ArgumentException($"{Id} robot systems must contain a {typeof(T).Name} robot arm.");

        return group;
    }
}

static class ManufacturerCatalog
{
    public static ManufacturerDefinition Get(Manufacturers manufacturer)
    {
        if (manufacturer == Manufacturers.All)
            throw new ArgumentException("Manufacturer 'All' is not a concrete manufacturer.", nameof(manufacturer));

        return Find(manufacturer) ?? throw Unsupported(manufacturer);
    }

    public static int GetControllerIOStartIndex(Manufacturers manufacturer) =>
        Find(manufacturer)?.ControllerIOStartIndex
        ?? throw new NotSupportedException($"Controller IO numbering is not supported for {manufacturer} robots.");

    static ManufacturerDefinition? Find(Manufacturers manufacturer) =>
        manufacturer switch
        {
            Manufacturers.ABB => AbbDefinition.Instance,
            Manufacturers.KUKA => KukaDefinition.Instance,
            Manufacturers.UR => UrDefinition.Instance,
            Manufacturers.Staubli => StaubliDefinition.Instance,
            Manufacturers.FrankaEmika => FrankaDefinition.Instance,
            Manufacturers.Doosan => DoosanDefinition.Instance,
            Manufacturers.Fanuc => FanucDefinition.Instance,
            Manufacturers.Igus => IgusDefinition.Instance,
            Manufacturers.Jaka => JakaDefinition.Instance,
            Manufacturers.All => null,
            _ => null
        };
}
