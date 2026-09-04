using GH_IO.Serialization;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

static class ParameterMigration
{
    public static void Input(IGH_Param source, IGH_Param target)
    {
        Copy(source, target);
        _ = GH_UpgradeUtil.MigrateSources(source, target);
    }

    public static void Output(IGH_Param source, IGH_Param target)
    {
        Copy(source, target);
        _ = GH_UpgradeUtil.MigrateRecipients(source, target);
    }

    static void Copy(IGH_Param source, IGH_Param target)
    {
        GH_LooseChunk archive = new("Parameter");

        if (!source.Write(archive))
            throw new InvalidOperationException($"Could not preserve parameter '{source.Name}'.");

        if (target is JointsParameter && source is not JointsParameter
            && archive.FindChunk("PersistentData") is { } persistent)
        {
            GH_Structure<IGH_Goo> values = [];

            if (!values.Read(persistent))
                throw new InvalidOperationException($"Could not read joint values from '{source.Name}'.");

            GH_Structure<GH_Joints> joints = [];

            for (int i = 0; i < values.PathCount; i++)
            {
                var path = values.Paths[i];
                var branch = values.Branches[i];
                _ = joints.EnsurePath(path);

                if (source.Access == GH_ParamAccess.list && branch.Count > 0
                    && branch.All(value => value is GH_Number or GH_Integer))
                {
                    var valuesInBranch = branch.Select(value => value is GH_Number number ? number.Value : ((GH_Integer)value).Value);
                    joints.Append(Param<double[], GH_Joints>.New([.. valuesInBranch]), path);
                }
                else
                {
                    foreach (var value in branch)
                    {
                        GH_Joints converted = new();

                        if (value is null || !converted.CastFrom(value))
                            throw new InvalidOperationException($"Could not convert a joint value in '{source.Name}'.");

                        joints.Append(converted, path);
                    }
                }
            }

            _ = archive.RemoveChunk("PersistentData");
            _ = joints.Write(archive.CreateChunk("PersistentData"));
        }

        var (name, description, access, optional) = (target.Name, target.Description, target.Access, target.Optional);

        if (!target.Read(archive))
            throw new InvalidOperationException($"Could not restore parameter '{source.Name}'.");

        target.Name = name;
        target.Description = description;
        target.Access = access;
        target.Optional = optional;
    }
}
