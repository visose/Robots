using SkiaSharp;

namespace Robots.Icons;

internal static class IconQa
{
    public static void ThrowIfInvalid(IReadOnlyList<IconOutput> outputs)
    {
        List<string> issues = [];

        foreach (var output in outputs)
        {
            using var bitmap = SKBitmap.Decode(output.Path);

            if (bitmap is null)
            {
                issues.Add($"{output.Name}: could not read PNG.");
                continue;
            }

            if (bitmap.Width != IconCanvas.OutputSize || bitmap.Height != IconCanvas.OutputSize)
            {
                issues.Add($"{output.Name}: expected {IconCanvas.OutputSize}x{IconCanvas.OutputSize}, got {bitmap.Width}x{bitmap.Height}.");
                continue;
            }

            if (TouchesEdge(bitmap))
            {
                issues.Add($"{output.Name}: non-transparent pixels touch the image edge.");
            }

            if (HasTransparentRgbLeak(bitmap))
            {
                issues.Add($"{output.Name}: transparent pixels contain RGB data.");
            }

            if (HasLowAlphaNeutralPixels(bitmap))
            {
                issues.Add($"{output.Name}: low-alpha neutral pixels remain.");
            }
        }

        if (issues.Count > 0)
        {
            throw new InvalidOperationException("Generated icon QA failed:" + Environment.NewLine + string.Join(Environment.NewLine, issues));
        }
    }

    static bool TouchesEdge(SKBitmap bitmap)
    {
        const byte threshold = 8;

        for (var x = 0; x < bitmap.Width; x++)
        {
            if (bitmap.GetPixel(x, 0).Alpha > threshold || bitmap.GetPixel(x, bitmap.Height - 1).Alpha > threshold)
            {
                return true;
            }
        }

        for (var y = 0; y < bitmap.Height; y++)
        {
            if (bitmap.GetPixel(0, y).Alpha > threshold || bitmap.GetPixel(bitmap.Width - 1, y).Alpha > threshold)
            {
                return true;
            }
        }

        return false;
    }

    static bool HasTransparentRgbLeak(SKBitmap bitmap)
    {
        for (var y = 0; y < bitmap.Height; y++)
        {
            for (var x = 0; x < bitmap.Width; x++)
            {
                var color = bitmap.GetPixel(x, y);

                if (color.Alpha == 0 && (color.Red != 0 || color.Green != 0 || color.Blue != 0))
                {
                    return true;
                }
            }
        }

        return false;
    }

    static bool HasLowAlphaNeutralPixels(SKBitmap bitmap)
    {
        for (var y = 0; y < bitmap.Height; y++)
        {
            for (var x = 0; x < bitmap.Width; x++)
            {
                var color = bitmap.GetPixel(x, y);
                var max = Math.Max(color.Red, Math.Max(color.Green, color.Blue));
                var min = Math.Min(color.Red, Math.Min(color.Green, color.Blue));

                if (color.Alpha is > 0 and < 70 && max > 160 && max - min < 70)
                {
                    return true;
                }
            }
        }

        return false;
    }
}
