using SkiaSharp;

namespace Robots.Icons;

internal static class ContactSheet
{
    public static void Write(IReadOnlyList<IconOutput> outputs, string path)
    {
        const int columns = 7;
        const int cell = 180;
        const int iconScale = 3;
        const int labelHeight = 26;

        if (outputs.Count == 0)
        {
            throw new ArgumentException("At least one icon is required.", nameof(outputs));
        }

        var rows = (outputs.Count + columns - 1) / columns;
        SKImageInfo info = new(columns * cell, rows * cell, SKColorType.Rgba8888, SKAlphaType.Premul);

        using var surface = SKSurface.Create(info) ?? throw new InvalidOperationException("Could not create contact sheet surface.");
        var canvas = surface.Canvas;
        canvas.Clear(SKColors.White);

        using SKPaint gridPaint = new()
        {
            Color = new(220, 220, 220),
            IsAntialias = false,
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 1
        };
        using SKPaint borderPaint = new()
        {
            Color = new(218, 232, 236),
            IsAntialias = false,
            Style = SKPaintStyle.Stroke,
            StrokeWidth = 1
        };
        using SKPaint textPaint = new()
        {
            Color = new(20, 20, 20),
            IsAntialias = true,
            Style = SKPaintStyle.Fill
        };
        using SKFont font = new(SKTypeface.Default, 9);

        for (var i = 0; i < outputs.Count; i++)
        {
            var output = outputs[i];
            var column = i % columns;
            var row = i / columns;
            var x = column * cell;
            var y = row * cell;
            var iconSize = IconCanvas.OutputSize * iconScale;
            var iconX = x + (cell - iconSize) / 2;
            var iconY = y + 14;
            SKRect iconBounds = new(iconX, iconY, iconX + iconSize, iconY + iconSize);

            using var icon = SKBitmap.Decode(output.Path) ?? throw new InvalidOperationException($"Could not read icon: {output.Path}");
            canvas.DrawBitmap(icon, iconBounds);
            canvas.DrawRect(iconBounds, borderPaint);
            canvas.DrawText(output.Name, x + 8, y + cell - labelHeight + 14, font, textPaint);

            if (column > 0)
            {
                canvas.DrawLine(x, y, x, y + cell, gridPaint);
            }

            if (row > 0)
            {
                canvas.DrawLine(x, y, x + cell, y, gridPaint);
            }
        }

        _ = Directory.CreateDirectory(Path.GetDirectoryName(path) ?? throw new InvalidOperationException($"Path has no parent directory: {path}"));

        using var image = surface.Snapshot();
        using var data = image.Encode(SKEncodedImageFormat.Png, 100) ?? throw new InvalidOperationException($"Could not encode contact sheet: {path}");
        using var stream = File.Create(path);
        data.SaveTo(stream);
    }
}
