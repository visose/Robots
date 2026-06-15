using System.Drawing;
using System.Drawing.Drawing2D;

using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;

namespace Robots.Grasshopper;

static class IconDrawing
{
    const int _iconSize = 24;

    public static void RenderScaledIcon(Graphics graphics, Image? icon, RectangleF box, int offsetY = 0)
    {
        if (icon is null || GH_Canvas.ZoomFadeLow < 5)
            return;

        var size = Math.Min(_iconSize, Math.Min(box.Width, box.Height));
        var x = Convert.ToInt32((box.Left + box.Right - size) * 0.5f);
        var y = Convert.ToInt32((box.Top + box.Bottom - size) * 0.5f) + offsetY;
        var target = new Rectangle(x, y, (int)size, (int)size);
        var state = graphics.Save();

        try
        {
            graphics.CompositingQuality = CompositingQuality.HighQuality;
            graphics.InterpolationMode = InterpolationMode.HighQualityBicubic;
            graphics.PixelOffsetMode = PixelOffsetMode.HighQuality;
            graphics.SmoothingMode = SmoothingMode.HighQuality;

            if (GH_Canvas.ZoomFadeLow >= 250)
            {
                graphics.DrawImage(icon, target, 0, 0, icon.Width, icon.Height, GraphicsUnit.Pixel);
            }
            else
            {
                GH_GraphicsUtil.RenderFadedImage(graphics, icon, target, GH_Canvas.ZoomFadeLow);
            }
        }
        finally
        {
            graphics.Restore(state);
        }
    }
}
