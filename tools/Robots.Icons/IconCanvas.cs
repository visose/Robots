using SkiaSharp;

namespace Robots.Icons;

internal static class IconPalette
{
    public static readonly SKColor Ink = IconCanvas.Color(38, 42, 44);
    public static readonly SKColor InkSoft = IconCanvas.Color(84, 89, 91);
    public static readonly SKColor MetalLight = IconCanvas.Color(226, 230, 228);
    public static readonly SKColor MetalDark = IconCanvas.Color(83, 91, 94);
    public static readonly SKColor Paper = IconCanvas.Color(247, 248, 242);
    public static readonly SKColor Orange = IconCanvas.Color(255, 132, 0);
    public static readonly SKColor OrangeLight = IconCanvas.Color(255, 205, 42);
    public static readonly SKColor OrangeDark = IconCanvas.Color(174, 66, 0);
    public static readonly SKColor Shadow = IconCanvas.Color(0, 0, 0, 72);
}

internal sealed class IconCanvas : IDisposable
{
    public const int DesignSize = 48;
    public const int OutputSize = 48;
    const int AntiAlias = 8;
    const int AlphaCutoff = 3;

    readonly SKSurface _surface;
    readonly SKCanvas _canvas;

    public IconCanvas()
    {
        SKImageInfo info = new(OutputSize * AntiAlias, OutputSize * AntiAlias, SKColorType.Rgba8888, SKAlphaType.Premul);
        _surface = SKSurface.Create(info) ?? throw new InvalidOperationException("Could not create Skia surface.");
        _canvas = _surface.Canvas;
        _canvas.Clear(SKColors.Transparent);
        _canvas.Scale(AntiAlias);
    }

    public void Dispose()
    {
        _surface.Dispose();
    }

    public SKBitmap Finish()
    {
        using var image = _surface.Snapshot();
        using var source = SKBitmap.FromImage(image);
        SKImageInfo outputInfo = new(OutputSize, OutputSize, SKColorType.Rgba8888, SKAlphaType.Premul);
        SKBitmap output = new(outputInfo);
        SKSamplingOptions sampling = new(SKFilterMode.Linear, SKMipmapMode.Linear);

        if (!source.ScalePixels(output, sampling))
        {
            output.Dispose();
            throw new InvalidOperationException("Could not downsample generated icon.");
        }

        CleanAlpha(output);
        return output;
    }

    public void Line(SKColor color, double width, params (double X, double Y)[] points)
    {
        if (points.Length < 2)
        {
            throw new ArgumentException("At least two points are required.", nameof(points));
        }

        using var paint = StrokePaint(color, width);
        using SKPath path = new();
        path.MoveTo(Point(points[0]));

        for (var i = 1; i < points.Length; i++)
        {
            path.LineTo(Point(points[i]));
        }

        _canvas.DrawPath(path, paint);
    }

    public void Polygon(SKColor fill, SKColor? outline = null, double width = 1.0, params (double X, double Y)[] points)
    {
        if (points.Length < 3)
        {
            throw new ArgumentException("At least three points are required.", nameof(points));
        }

        using var path = ClosedPath(points);

        using (var paint = FillPaint(fill))
        {
            _canvas.DrawPath(path, paint);
        }

        if (outline is { } outlineColor)
        {
            using var paint = StrokePaint(outlineColor, width);
            _canvas.DrawPath(path, paint);
        }
    }

    public void Rect((double Left, double Top, double Right, double Bottom) bounds, SKColor fill, SKColor? outline = null, double width = 1.0)
    {
        var rect = Rect(bounds);

        using (var paint = FillPaint(fill))
        {
            _canvas.DrawRect(rect, paint);
        }

        if (outline is { } outlineColor)
        {
            using var paint = StrokePaint(outlineColor, width);
            _canvas.DrawRect(rect, paint);
        }
    }

    public void Rounded((double Left, double Top, double Right, double Bottom) bounds, double radius, SKColor fill, SKColor? outline = null, double width = 1.0)
    {
        var rect = Rect(bounds);

        using (var paint = FillPaint(fill))
        {
            _canvas.DrawRoundRect(rect, (float)radius, (float)radius, paint);
        }

        if (outline is { } outlineColor)
        {
            using var paint = StrokePaint(outlineColor, width);
            _canvas.DrawRoundRect(rect, (float)radius, (float)radius, paint);
        }
    }

    public void Ellipse((double Left, double Top, double Right, double Bottom) bounds, SKColor? fill = null, SKColor? outline = null, double width = 1.0)
    {
        var rect = Rect(bounds);

        if (fill is { } fillColor)
        {
            using var paint = FillPaint(fillColor);
            _canvas.DrawOval(rect, paint);
        }

        if (outline is { } outlineColor)
        {
            using var paint = StrokePaint(outlineColor, width);
            _canvas.DrawOval(rect, paint);
        }
    }

    public void Arc((double Left, double Top, double Right, double Bottom) bounds, double start, double end, SKColor color, double width = 1.0)
    {
        using var paint = StrokePaint(color, width);
        _canvas.DrawArc(Rect(bounds), (float)start, (float)(end - start), false, paint);
    }

    public void DrawPath(SKPath path, SKColor fill, SKColor? outline = null, double width = 1.0)
    {
        using (var paint = FillPaint(fill))
        {
            _canvas.DrawPath(path, paint);
        }

        if (outline is { } outlineColor)
        {
            using var paint = StrokePaint(outlineColor, width);
            _canvas.DrawPath(path, paint);
        }
    }

    public static (double X, double Y) P(double x, double y) => (x, y);

    public static SKColor Color(byte red, byte green, byte blue, byte alpha = 255)
    {
        return new(red, green, blue, alpha);
    }

    public static (double X, double Y)[] Points(params double[] coordinates)
    {
        if (coordinates.Length == 0 || coordinates.Length % 2 != 0)
        {
            throw new ArgumentException("Coordinates must contain one or more x/y pairs.", nameof(coordinates));
        }

        var points = new (double X, double Y)[coordinates.Length / 2];

        for (var i = 0; i < points.Length; i++)
        {
            var j = i * 2;
            points[i] = (coordinates[j], coordinates[j + 1]);
        }

        return points;
    }

    public static (double X, double Y)[] Offset((double X, double Y)[] points, double dx, double dy)
    {
        var shifted = new (double X, double Y)[points.Length];

        for (var i = 0; i < points.Length; i++)
        {
            shifted[i] = (points[i].X + dx, points[i].Y + dy);
        }

        return shifted;
    }

    public static void SavePng(SKBitmap bitmap, string path)
    {
        _ = Directory.CreateDirectory(Path.GetDirectoryName(path) ?? throw new InvalidOperationException($"Path has no parent directory: {path}"));

        using var image = SKImage.FromBitmap(bitmap);
        using var data = image.Encode(SKEncodedImageFormat.Png, 100) ?? throw new InvalidOperationException($"Could not encode PNG: {path}");
        using var stream = File.Create(path);
        data.SaveTo(stream);
    }

    static void CleanAlpha(SKBitmap bitmap)
    {
        for (var y = 0; y < bitmap.Height; y++)
        {
            for (var x = 0; x < bitmap.Width; x++)
            {
                var color = bitmap.GetPixel(x, y);

                if (color.Alpha <= AlphaCutoff)
                {
                    bitmap.SetPixel(x, y, SKColors.Transparent);
                    continue;
                }

                var max = Math.Max(color.Red, Math.Max(color.Green, color.Blue));
                var min = Math.Min(color.Red, Math.Min(color.Green, color.Blue));

                if (color.Alpha < 70 && max > 160 && max - min < 70)
                {
                    bitmap.SetPixel(x, y, SKColors.Transparent);
                }
            }
        }
    }

    static SKPath ClosedPath((double X, double Y)[] points)
    {
        SKPath path = new();
        path.MoveTo(Point(points[0]));

        for (var i = 1; i < points.Length; i++)
        {
            path.LineTo(Point(points[i]));
        }

        path.Close();
        return path;
    }

    static SKPoint Point((double X, double Y) point) => new((float)point.X, (float)point.Y);

    static SKRect Rect((double Left, double Top, double Right, double Bottom) bounds)
    {
        return new((float)bounds.Left, (float)bounds.Top, (float)bounds.Right, (float)bounds.Bottom);
    }

    static SKPaint FillPaint(SKColor color)
    {
        return new()
        {
            Color = color,
            IsAntialias = true,
            Style = SKPaintStyle.Fill
        };
    }

    static SKPaint StrokePaint(SKColor color, double width)
    {
        return new()
        {
            Color = color,
            IsAntialias = true,
            Style = SKPaintStyle.Stroke,
            StrokeWidth = (float)width,
            StrokeCap = SKStrokeCap.Round,
            StrokeJoin = SKStrokeJoin.Round
        };
    }
}
