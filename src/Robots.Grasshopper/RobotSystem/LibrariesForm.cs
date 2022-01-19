using Eto.Drawing;
using Eto.Forms;

namespace Robots.Grasshopper;

class LibrariesForm : ComponentForm
{
    public LibrariesForm()
    {
        Title = "Robot libraries";

        Padding = new Padding(5);
        MinimumSize = new Size(200, 200);

        Content = new StackLayout
        {
            Padding = 10,
            Items =
                {
                    "Hello World!"
                }
        };
    }
}