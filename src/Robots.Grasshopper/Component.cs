using System.Drawing;

namespace Robots.Grasshopper;

sealed class MissingInputException() : Exception("Input data is missing.");

sealed class RuntimeWarningException(string message) : Exception(message);

public abstract class Component(
    string name,
    string nickname,
    string description,
    string subcategory,
    string id,
    GH_Exposure exposure = GH_Exposure.primary)
    : GH_Component(name, nickname, description, "Robots", subcategory)
{
    protected Component(
        string name,
        string description,
        string subcategory,
        string id,
        GH_Exposure exposure = GH_Exposure.primary)
        : this(name, InferNickname(name), description, subcategory, id, exposure) { }

    public override GH_Exposure Exposure => exposure;
    public override Guid ComponentGuid => new(id);
    public override bool IsPreviewCapable => Params.Output.Any(static param =>
        param is IGH_PreviewObject { IsPreviewCapable: true });
    protected override Bitmap Icon => Util.GetIcon(GetType());

    public override void CreateAttributes()
    {
        m_attributes = new ComponentAttributes(this);
    }

    protected override void RegisterInputParams(GH_InputParamManager pManager) { }
    protected override void RegisterOutputParams(GH_OutputParamManager pManager) { }
    protected abstract void SolveComponent(IGH_DataAccess DA);

    protected sealed override void SolveInstance(IGH_DataAccess DA)
    {
        try
        {
            SolveComponent(DA);
        }
        catch (MissingInputException)
        {
            DA.AbortComponentSolution();
        }
        catch (Exception e)
        {
            var level = e is RuntimeWarningException
                ? GH_RuntimeMessageLevel.Warning
                : GH_RuntimeMessageLevel.Error;

            AddRuntimeMessage(level, RuntimeMessage(e));
            DA.AbortComponentSolution();
        }
    }

    static string InferNickname(string name) => string.Concat(name.Split(' ', StringSplitOptions.RemoveEmptyEntries));

    static string RuntimeMessage(Exception exception)
    {
        var message = exception is System.Xml.XmlException xml
            ? $"Invalid XML syntax in \"{Path.GetFileName(xml.SourceUri)}\"."
            : exception.Message;

        return CleanArgumentMessage(message);
    }

    static string CleanArgumentMessage(string message)
    {
        var lines = message.Replace("\r\n", "\n").Split('\n');
        var filtered = lines
            .Where(line =>
                !line.StartsWith("Parameter ", StringComparison.Ordinal)
                && !line.StartsWith("Actual value was ", StringComparison.Ordinal))
            .ToArray();

        return string.Join(Environment.NewLine, filtered);
    }
}
