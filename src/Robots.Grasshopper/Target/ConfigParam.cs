using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Special;
using Grasshopper.Kernel.Types;

namespace Robots.Grasshopper;

public class ConfigParam : GH_ValueList
{
    public override string Name => "Flag Fields";
    public override string Description => "Combines selected robot configuration flags.";
    public override Guid ComponentGuid => new("{0381B555-BF9C-4D68-8E5C-10B2FCB16F30}");
    public override GH_Exposure Exposure => GH_Exposure.hidden;

    protected override void OnVolatileDataCollected()
    {
        int config = 0;

        if (VolatileDataCount > 0)
        {
            var values = VolatileData.get_Branch(0);

            foreach (var value in values)
            {
                if (value is GH_Integer integer)
                {
                    config |= integer.Value;
                }
            }
        }

        VolatileData.Clear();
        _ = AddVolatileData(new GH_Path(0), 0, new GH_Integer(config));
    }
}
