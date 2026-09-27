namespace Robots.Grasshopper;

static class ProgramToolpaths
{
    internal static int[] TargetInputIndices(GH_ComponentParamServer parameters)
    {
        return [.. parameters.Input.Select((param, index) => (param, index)).Where(x => IsTargetInput(x.param)).Select(x => x.index)];
    }

    internal static bool IsTargetInput(IGH_Param param) => param is ToolpathParameter or TargetParameter;

    internal static IToolpath[] Read(IGH_DataAccess DA, GH_ComponentParamServer parameters)
    {
        var indices = TargetInputIndices(parameters);
        var toolpaths = new IToolpath[indices.Length];

        for (int i = 0; i < indices.Length; i++)
        {
            int index = indices[i];
            var param = parameters.Input[index];

            try
            {
                toolpaths[i] = ReadToolpath(DA, index, param);
            }
            catch (MissingInputException)
            {
                throw new RuntimeWarningException($"Input parameter {param.NickName} failed to collect data.");
            }
        }

        return toolpaths;
    }

    static SimpleToolpath ReadToolpath(IGH_DataAccess DA, int index, IGH_Param param)
    {
        if (param is ToolpathParameter)
            return new(DA.List<IToolpath>(index));

        return new(DA.List<Target>(index));
    }
}
