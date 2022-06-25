using System.Windows.Controls;
using Autodesk.DesignScript.Runtime;
using Dynamo.Wpf.Extensions;
using Dynamo.Controls;
using HelixToolkit.Wpf.SharpDX;

namespace Robots.Dynamo;

[IsVisibleInDynamoLibrary(false)]
public class RobotsViewExtension : IViewExtension
{
    internal static RobotsViewExtension Instance { get; private set; } = default!;
    
    Panel _content = default!;
    public string UniqueId => "073FBA9B-49C6-41D1-95FD-8335D6E8F305";
    public string Name => nameof(RobotsViewExtension);

    public void Startup(ViewStartupParams viewStartupParams) { }

    public void Shutdown() { }

    public void Loaded(ViewLoadedParams viewLoadedParams)
    {
        var window = viewLoadedParams.DynamoWindow;
        _content = (Panel)window.Content;
        Instance = this;
    }

    Viewport3DX GetViewport()
    {
        Queue<object> queue = new();

        foreach (var item in _content.Children)
            queue.Enqueue(item);

        while (queue.Any())
        {
            var child = queue.Dequeue();

            if (child is Watch3DView watch3Dview)
                return watch3Dview.View;

            if (child is Panel next)
            {
                foreach (var item in next.Children)
                    queue.Enqueue(item);
            }
        }

        throw new InvalidOperationException("Could not find Viewport3DX");
    }

    internal void AttachModels(List<MeshGeometryModel3D> models)
    {
        var viewport = GetViewport();

        foreach (var model in models)
            viewport.Items.Add(model);
    }

    public void Dispose() { }
}
