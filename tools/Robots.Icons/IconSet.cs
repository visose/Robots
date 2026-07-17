using SkiaSharp;

using static Robots.Icons.IconCanvas;
using static Robots.Icons.IconPalette;

namespace Robots.Icons;

internal static class IconSet
{
    static readonly string[] IconNames =
    [
        "CheckCollisions",
        "CommandParameter",
        "CreateFrame",
        "CreateProgramVariable",
        "CreateSpeedAccel",
        "CreateTarget",
        "CreateTool",
        "CustomCode",
        "CustomCommand",
        "DeconstructProgramTargets",
        "DeconstructTarget",
        "DeconstructTool",
        "DeconstructToolpath",
        "DegreesToRadians",
        "DrawSimpleTrail",
        "FrameParameter",
        "FromPlane",
        "GetPlane",
        "Group",
        "JointsParameter",
        "Kinematics",
        "LoadFrame",
        "LoadRobotSystem",
        "LoadTool",
        "Message",
        "PostProcessorParameter",
        "ProgramParameter",
        "PulseDO",
        "Remote",
        "RobotSystemParameter",
        "SaveProgram",
        "SetAO",
        "SetDO",
        "Simulation",
        "SpeedParameter",
        "Stop",
        "TargetParameter",
        "ToolParameter",
        "ToolpathParameter",
        "Wait",
        "WaitDI",
        "ZoneParameter",
    ];

    public static IReadOnlyList<IconOutput> WriteIcons(string iconsDirectory)
    {
        _ = Directory.CreateDirectory(iconsDirectory);
        List<IconOutput> outputs = new(IconNames.Length);

        foreach (var name in IconNames)
        {
            using var bitmap = DrawIcon(name);
            var path = Path.Combine(iconsDirectory, $"{name}.png");
            SavePng(bitmap, path);
            outputs.Add(new IconOutput(name, path));
        }

        return outputs;
    }

    static SKBitmap DrawIcon(string name)
    {
        return name.EndsWith("Parameter", StringComparison.Ordinal)
            ? DrawParameter(name)
            : DrawComponent(name);
    }

    static SKBitmap DrawParameter(string name)
    {
        using var g = new IconCanvas();
        DrawHex(g);
        var inner = name[..^"Parameter".Length];

        switch (inner)
        {
            case "Command":
                g.Line(Orange, 3.0, P(17, 19), P(23, 24), P(17, 29));
                g.Line(Orange, 2.3, P(25, 30), P(33, 30));
                break;
            case "Frame":
                DrawAxes(g, 23.5, 30.5, 13.0);
                break;
            case "Joints":
                g.Line(Ink, 4.4, P(15, 30), P(22, 21), P(30, 19), P(36, 26));
                g.Line(Orange, 2.5, P(15, 30), P(22, 21), P(30, 19), P(36, 26));

                foreach (var (x, y) in new[] { P(15, 30), P(22, 21), P(30, 19), P(36, 26) })
                {
                    g.Ellipse((x - 2.6, y - 2.6, x + 2.6, y + 2.6), OrangeLight, Ink, 0.9);
                }

                break;
            case "PostProcessor":
                DrawPage(g, 14, 10, 20, 25, lines: false);
                g.Ellipse((18.8, 19.8, 29.2, 30.2), Orange, Ink, 1.0);
                g.Ellipse((22.2, 23.2, 25.8, 26.8), Paper, Ink, 0.65);
                break;
            case "Program":
                DrawPage(g, 14, 10, 20, 25, lines: false);
                g.Line(Ink, 2.5, P(19, 20), P(24, 24), P(19, 28));
                g.Line(Orange, 2.1, P(26, 29), P(31, 29));
                break;
            case "RobotSystem":
                DrawRobot(g, 8, 7.5, 0.72);
                break;
            case "Speed":
                DrawGauge(g, 24, 29, 12);
                break;
            case "Target":
                DrawCrosshair(g, 24, 24, 9.5, shadow: false);
                break;
            case "Tool":
                DrawGripper(g, 24, 21, 0.72);
                break;
            case "Toolpath":
                DrawWave(g, Points(15, 31, 19, 23, 25, 24, 31, 18, 36, 20), Orange, 2.7);

                foreach (var (x, y) in new[] { P(15, 31), P(25, 24), P(36, 20) })
                {
                    g.Ellipse((x - 2.3, y - 2.3, x + 2.3, y + 2.3), OrangeLight, Ink, 0.75);
                }

                break;
            case "Zone":
                g.Line(Ink, 3.8, P(15, 31), P(23, 25), P(34, 27));
                g.Line(MetalLight, 1.8, P(15, 31), P(23, 25), P(34, 27));
                g.Arc((13, 14, 35, 36), 25, 155, Ink, 3.6);
                g.Arc((13, 14, 35, 36), 205, 335, Ink, 3.6);
                g.Arc((13, 14, 35, 36), 25, 155, Orange, 2.2);
                g.Arc((13, 14, 35, 36), 205, 335, Orange, 2.2);
                g.Ellipse((20.5, 22.5, 25.5, 27.5), OrangeLight, Ink, 0.7);
                break;
            default:
                throw new InvalidOperationException($"No parameter icon drawing exists for {name}.");
        }

        return g.Finish();
    }

    static SKBitmap DrawComponent(string name)
    {
        using var g = new IconCanvas();

        switch (name)
        {
            case "CheckCollisions":
                DrawRobot(g, 1, 3, 1.0);
                g.Rounded((27, 23, 39, 35), 3, MetalLight, Ink, 1.2);
                DrawCheckBadge(g);
                break;
            case "CreateFrame":
                DrawAxes(g, 17, 32, 18);
                break;
            case "CreateProgramVariable":
                DrawPromptDocument(g, 11, 6, 24, 31, promptDx: -1.5, promptDy: -3.0);
                DrawPlusBadge(g);
                break;
            case "CreateSpeedAccel":
                DrawGauge(g, 22, 29, 16);
                break;
            case "CreateTarget":
                DrawCrosshair(g, 21, 21, 13);
                DrawPlusBadge(g);
                break;
            case "CreateTool":
                DrawGripper(g, 22, 20, 0.95);
                DrawPlusBadge(g);
                break;
            case "CustomCode":
                DrawTerminal(g, 7, 10, 34, 27);
                break;
            case "CustomCommand":
                DrawDocument(g, 10, 6, 24, 31);
                break;
            case "DeconstructProgramTargets":
                DrawProgramTargetsPanel(g);
                break;
            case "DeconstructTarget":
                DrawCrosshair(g, 18, 22, 11);
                DrawDeconstructPorts(g);
                break;
            case "DeconstructTool":
                DrawGripper(g, 18, 20, 0.9);
                DrawDeconstructPorts(g);
                break;
            case "DeconstructToolpath":
                DrawWave(g, Points(8, 31, 16, 21, 23, 28, 31, 15), MetalLight, 3.0);
                DrawDeconstructPorts(g);
                break;
            case "DegreesToRadians":
                DrawDegreesToRadians(g);
                break;
            case "DrawSimpleTrail":
                DrawSimpleTrail(g);
                break;
            case "FromPlane":
                DrawPlaneAxes(g, 5, 33, 27);
                DrawDeconstructPorts(g);
                break;
            case "GetPlane":
                DrawConstructPorts(g);
                DrawPlaneAxes(g, 17, 33, 27);
                break;
            case "Group":
                foreach (var (order, x, y) in new[] { (0, 10.0, 25.0), (1, 16.0, 17.0), (2, 22.0, 9.0) })
                {
                    var fill = order == 2 ? Orange : Color(180, 187, 188);
                    g.Rounded((x, y, x + 14, y + 12), 2, fill, Ink, 1.2);
                }

                break;
            case "Kinematics":
                DrawRobot(g, 2, 5, 0.96);
                DrawWave(g, Points(27, 33, 32, 29, 37, 27, 42, 24), Orange, 2.2);
                break;
            case "LoadFrame":
                DrawAxes(g, 16, 32, 16);
                DrawLoadBadge(g);
                break;
            case "LoadRobotSystem":
                DrawRobot(g, 1, 4, 0.98);
                DrawLoadBadge(g);
                break;
            case "LoadTool":
                DrawGripper(g, 22, 20, 0.95);
                DrawLoadBadge(g);
                break;
            case "Message":
                DrawMessage(g);
                break;
            case "PulseDO":
                DrawWave(g, Points(8, 30, 8, 19, 18, 19, 18, 30, 30, 30, 30, 19, 40, 19, 40, 30), Orange, 2.8);
                break;
            case "Remote":
                g.Line(Ink, 3.0, P(24, 38), P(24, 22));
                g.Polygon(MetalDark, Ink, 1.1, P(18, 39), P(30, 39), P(27, 32), P(21, 32));
                g.Ellipse((20, 17, 28, 25), Orange, Ink, 1.2);

                foreach (var offset in new[] { 7.0, 11.0 })
                {
                    g.Arc((24 - offset, 16 - offset, 24 + offset, 32 + offset), 210, 330, Orange, 2.0);
                    g.Arc((24 - offset, 16 - offset, 24 + offset, 32 + offset), 210, 330, Ink, 0.7);
                }

                break;
            case "SaveProgram":
                DrawFloppy(g);
                break;
            case "SetAO":
                g.Line(Ink, 1.5, P(9, 38), P(9, 13));
                DrawWave(g, Points(9, 28, 15, 17, 22, 29, 29, 32, 37, 20, 42, 27), Orange, 2.6);
                break;
            case "SetDO":
                g.Line(Ink, 1.5, P(8, 38), P(8, 13));
                DrawWave(g, Points(8, 30, 8, 20, 18, 20, 18, 30, 31, 30, 31, 20, 41, 20), Orange, 2.6);
                break;
            case "Simulation":
                DrawRobot(g, 2, 5, 0.96);
                DrawPlayBadge(g);
                break;
            case "Stop":
                DrawStop(g);
                break;
            case "Wait":
                DrawClock(g, 24, 24, 15);
                break;
            case "WaitDI":
                DrawWaitDi(g);
                break;
            default:
                throw new InvalidOperationException($"No component icon drawing exists for {name}.");
        }

        return g.Finish();
    }

    static void DrawArrow(IconCanvas g, (double X, double Y) start, (double X, double Y) end, SKColor color, double width = 2.8)
    {
        var dx = end.X - start.X;
        var dy = end.Y - start.Y;
        var length = Math.Max(Math.Sqrt(dx * dx + dy * dy), 0.01);
        var ux = dx / length;
        var uy = dy / length;
        var px = -uy;
        var py = ux;
        const double head = 4.8;
        const double side = 3.5;
        var shaftEnd = P(end.X - ux * head * 0.72, end.Y - uy * head * 0.72);

        g.Line(Shadow, width, P(start.X + 0.8, start.Y + 0.8), P(shaftEnd.X + 0.8, shaftEnd.Y + 0.8));
        g.Line(Ink, width + 1.8, start, shaftEnd);
        g.Line(color, width, start, shaftEnd);
        g.Polygon(
            color,
            Ink,
            0.8,
            end,
            P(end.X - ux * head + px * side, end.Y - uy * head + py * side),
            P(end.X - ux * head - px * side, end.Y - uy * head - py * side));
    }

    static (double Cx, double Cy) DrawBadgeBase(IconCanvas g)
    {
        const double cx = 36.0;
        const double cy = 35.0;
        const double radius = 7.4;
        g.Ellipse((cx - radius + 1.1, cy - radius + 1.1, cx + radius + 1.1, cy + radius + 1.1), Shadow);
        g.Ellipse((cx - radius, cy - radius, cx + radius, cy + radius), Orange, Ink, 1.4);
        g.Arc((cx - radius + 1.6, cy - radius + 1.6, cx + radius - 1.6, cy + radius - 1.6), 205, 320, OrangeLight, 1.1);
        return (cx, cy);
    }

    static void DrawPlusBadge(IconCanvas g)
    {
        var (cx, cy) = DrawBadgeBase(g);
        g.Line(Ink, 1.7, P(cx - 3.4, cy), P(cx + 3.4, cy));
        g.Line(Ink, 1.7, P(cx, cy - 3.4), P(cx, cy + 3.4));
    }

    static void DrawPlayBadge(IconCanvas g)
    {
        var (cx, cy) = DrawBadgeBase(g);
        g.Polygon(Ink, points: [P(cx - 2.0, cy - 4.2), P(cx - 2.0, cy + 4.2), P(cx + 4.8, cy)]);
    }

    static void DrawCheckBadge(IconCanvas g)
    {
        var (cx, cy) = DrawBadgeBase(g);
        g.Line(Ink, 2.1, P(cx - 3.8, cy + 0.3), P(cx - 1.1, cy + 3.1), P(cx + 4.4, cy - 3.5));
    }

    static void DrawLoadBadge(IconCanvas g)
    {
        var (cx, cy) = DrawBadgeBase(g);
        g.Line(Ink, 1.8, P(cx, cy - 4.1), P(cx, cy + 1.2));
        g.Polygon(Ink, points: [P(cx - 3.3, cy), P(cx + 3.3, cy), P(cx, cy + 4.1)]);
    }

    static void DrawHex(IconCanvas g)
    {
        var points = Points(13.5, 5.5, 34.5, 5.5, 45.0, 24.0, 34.5, 42.5, 13.5, 42.5, 3.0, 24.0);
        g.Polygon(Shadow, points: Offset(points, 1.1, 1.1));
        g.Polygon(Color(31, 38, 42), Color(61, 68, 71), 2.1, points);
        g.Line(Color(172, 181, 181, 225), 0.95, points[0], points[1], points[2]);
    }

    static void DrawAxes(IconCanvas g, double cx = 18.0, double cy = 31.0, double size = 17.0)
    {
        DrawArrow(g, P(cx, cy), P(cx + size, cy), Orange);
        DrawArrow(g, P(cx, cy), P(cx, cy - size), Orange);
        DrawArrow(g, P(cx, cy), P(cx - size * 0.68, cy + size * 0.45), MetalDark, 2.6);
        g.Ellipse((cx - 4.0, cy - 4.0, cx + 4.0, cy + 4.0), OrangeLight, Ink, 1.2);
    }

    static void DrawRobot(IconCanvas g, double ox = 6.0, double oy = 7.0, double scale = 1.0)
    {
        (double X, double Y) T((double X, double Y) point)
        {
            return P(ox + point.X * scale, oy + point.Y * scale);
        }

        g.Ellipse((ox + 4 * scale, oy + 31 * scale, ox + 29 * scale, oy + 40 * scale), Shadow);
        g.Rounded((ox + 6 * scale, oy + 30 * scale, ox + 28 * scale, oy + 38 * scale), 3 * scale, MetalDark, Ink, 1.3 * scale);
        g.Rounded((ox + 10 * scale, oy + 23 * scale, ox + 24 * scale, oy + 32 * scale), 2 * scale, Orange, Ink, 1.2 * scale);

        var path = Points(17, 25, 21, 14, 31, 9, 36, 18);

        for (var i = 0; i < path.Length; i++)
        {
            path[i] = T(path[i]);
        }

        g.Line(Ink, 7.2 * scale, path);
        g.Line(MetalLight, 4.9 * scale, path);
        g.Line(Orange, 3.0 * scale, T(P(21, 14)), T(P(31, 9)));

        var wrist = T(P(36, 18));
        var link = T(P(40, 18));
        g.Line(Ink, 3.2 * scale, wrist, link);
        g.Line(MetalLight, 1.7 * scale, wrist, link);
        g.Line(Ink, 2.0 * scale, link, T(P(43, 14)));
        g.Line(Ink, 2.0 * scale, link, T(P(43, 22)));

        foreach (var (x, y, radius) in new[] { (17.0, 25.0, 4.5), (21.0, 14.0, 4.3), (31.0, 9.0, 4.1), (36.0, 18.0, 3.3) })
        {
            var (cx, cy) = T(P(x, y));
            g.Ellipse((cx - radius * scale, cy - radius * scale, cx + radius * scale, cy + radius * scale), MetalLight, Ink, 1.2 * scale);
        }
    }

    static void DrawGripper(IconCanvas g, double cx = 24.0, double cy = 21.0, double scale = 1.0)
    {
        var s = scale;
        g.Rounded((cx - 5 * s, cy - 15 * s, cx + 5 * s, cy - 5 * s), 1.8 * s, MetalLight, Ink, 1.3 * s);
        g.Rounded((cx - 10 * s, cy - 6 * s, cx + 10 * s, cy + 2 * s), 2.0 * s, MetalDark, Ink, 1.4 * s);
        g.Line(Ink, 5.0 * s, P(cx - 7 * s, cy + 1 * s), P(cx - 14 * s, cy + 11 * s), P(cx - 10 * s, cy + 17 * s));
        g.Line(Ink, 5.0 * s, P(cx + 7 * s, cy + 1 * s), P(cx + 14 * s, cy + 11 * s), P(cx + 10 * s, cy + 17 * s));
        g.Line(MetalLight, 2.7 * s, P(cx - 7 * s, cy + 1 * s), P(cx - 14 * s, cy + 11 * s), P(cx - 10 * s, cy + 17 * s));
        g.Line(MetalLight, 2.7 * s, P(cx + 7 * s, cy + 1 * s), P(cx + 14 * s, cy + 11 * s), P(cx + 10 * s, cy + 17 * s));
    }

    static void DrawDocument(IconCanvas g, double x = 10, double y = 6, double w = 24, double h = 32)
    {
        DrawPage(g, x, y, w, h, lines: true);
    }

    static void DrawPage(IconCanvas g, double x, double y, double w, double h, bool lines)
    {
        g.Polygon(
            Shadow,
            points:
            [
                P(x + 1.2, y + 1.2),
                P(x + w - 5.8, y + 1.2),
                P(x + w + 1.2, y + 8.2),
                P(x + w + 1.2, y + h + 1.2),
                P(x + 1.2, y + h + 1.2)
            ]);
        g.Polygon(Paper, Ink, 1.5, P(x, y), P(x + w - 7, y), P(x + w, y + 7), P(x + w, y + h), P(x, y + h));
        g.Polygon(MetalLight, Ink, 1.0, P(x + w - 7, y), P(x + w - 7, y + 7), P(x + w, y + 7));

        if (!lines)
        {
            return;
        }

        foreach (var yy in new[] { y + 13, y + 20, y + 27 })
        {
            g.Line(MetalDark, 1.6, P(x + 5, yy), P(x + w - 5, yy));
        }
    }

    static void DrawPromptDocument(
        IconCanvas g,
        double x = 10,
        double y = 6,
        double w = 24,
        double h = 32,
        double promptDx = 0,
        double promptDy = 0)
    {
        DrawPage(g, x, y, w, h, lines: false);
        var px = x + promptDx;
        var py = y + promptDy;
        g.Line(Ink, 2.6, P(px + 7, py + 14), P(px + 12, py + 18), P(px + 7, py + 22));
        g.Line(Orange, 2.3, P(px + 16, py + 23), P(px + w - 5, py + 23));
    }

    static void DrawTerminal(IconCanvas g, double x = 8, double y = 10, double w = 32, double h = 26)
    {
        g.Rounded((x + 1.1, y + 1.1, x + w + 1.1, y + h + 1.1), 2.5, Shadow);
        g.Rounded((x, y, x + w, y + h), 2.5, Color(28, 35, 38), Ink, 1.7);
        g.Line(Orange, 3.0, P(x + 9, y + 8), P(x + 16, y + 13), P(x + 9, y + 18));
        g.Line(Orange, 2.4, P(x + 20, y + 20), P(x + 29, y + 20));
    }

    static void DrawCrosshair(IconCanvas g, double cx = 22.0, double cy = 21.5, double radius = 13.0, bool shadow = true)
    {
        if (shadow)
        {
            g.Ellipse((cx - radius + 1, cy - radius + 1, cx + radius + 1, cy + radius + 1), Shadow);
        }

        g.Ellipse((cx - radius, cy - radius, cx + radius, cy + radius), null, Ink, 2.2);
        g.Ellipse((cx - radius + 2.8, cy - radius + 2.8, cx + radius - 2.8, cy + radius - 2.8), null, Orange, 2.1);
        g.Line(Ink, 1.6, P(cx - radius - 4, cy), P(cx + radius + 4, cy));
        g.Line(Ink, 1.6, P(cx, cy - radius - 4), P(cx, cy + radius + 4));
        g.Ellipse((cx - 3, cy - 3, cx + 3, cy + 3), MetalLight, Ink, 1.1);
    }

    static void DrawGauge(IconCanvas g, double cx = 24, double cy = 28, double radius = 16)
    {
        g.Arc((cx - radius + 1, cy - radius + 1, cx + radius + 1, cy + radius + 1), 190, 350, Shadow, 4.0);
        g.Arc((cx - radius, cy - radius, cx + radius, cy + radius), 190, 350, Ink, 4.4);
        g.Arc((cx - radius, cy - radius, cx + radius, cy + radius), 190, 315, Orange, 2.8);

        foreach (var angle in new[] { 205, 235, 265, 295, 325 })
        {
            var radians = Math.PI * angle / 180.0;
            var p1 = P(cx + (radius - 5) * Math.Cos(radians), cy + (radius - 5) * Math.Sin(radians));
            var p2 = P(cx + (radius - 2) * Math.Cos(radians), cy + (radius - 2) * Math.Sin(radians));
            g.Line(Ink, 1.2, p1, p2);
        }

        g.Line(Ink, 3.5, P(cx, cy), P(cx + 9, cy - 8));
        g.Line(MetalDark, 2.0, P(cx, cy), P(cx + 9, cy - 8));
        g.Ellipse((cx - 3, cy - 3, cx + 3, cy + 3), MetalLight, Ink, 1.0);
    }

    static void DrawPlaneAxes(IconCanvas g, double x = 7, double y = 31, double width = 29)
    {
        var origin = P(x + 8, y + 5);
        DrawArrow(g, origin, P(origin.X + width * 0.72, origin.Y - 4), Orange, 2.1);
        DrawArrow(g, origin, P(origin.X + 2, origin.Y - 16), Orange, 2.1);
        g.Ellipse((origin.X - 2.5, origin.Y - 2.5, origin.X + 2.5, origin.Y + 2.5), OrangeLight, Ink, 0.75);
    }

    static void DrawDeconstructPorts(IconCanvas g)
    {
        DrawPorts(g, x: 38, mirror: false);
    }

    static void DrawConstructPorts(IconCanvas g)
    {
        DrawPorts(g, x: 10, mirror: true);
    }

    static void DrawPorts(IconCanvas g, double x, bool mirror)
    {
        const double radius = 2.45;

        foreach (var y in new[] { 14.0, 24.0, 34.0 })
        {
            if (mirror)
            {
                g.Line(InkSoft, 1.35, P(x + radius, y), P(x + 8.5, y));
            }
            else
            {
                g.Line(InkSoft, 1.35, P(x - 8.5, y), P(x - radius, y));
            }

            g.Ellipse((x - radius, y - radius, x + radius, y + radius), Orange, Ink, 0.95);
        }
    }

    static void DrawProgramTargetsPanel(IconCanvas g)
    {
        g.Rounded((8.5, 7.5, 30.5, 39.5), 3, Shadow);
        g.Rounded((7, 6, 29, 38), 3, MetalDark, Ink, 1.5);
        g.Rounded((10, 9, 26, 17), 1.6, Orange, Ink, 0.9);
        g.Rounded((10, 20, 26, 28), 1.6, Paper, Ink, 0.9);
        g.Rounded((10, 31, 26, 35), 1.4, MetalLight, Ink, 0.8);
        g.Line(OrangeLight, 1.0, P(12, 11), P(24, 11));
        DrawDeconstructPorts(g);
    }

    static void DrawDegreesToRadians(IconCanvas g)
    {
        var pivot = P(15, 34);
        g.Ellipse((pivot.X - 3.2, pivot.Y - 3.2, pivot.X + 3.2, pivot.Y + 3.2), OrangeLight, Ink, 0.9);
        g.Line(Ink, 2.0, pivot, P(39, 34));
        g.Line(Ink, 2.0, pivot, P(30, 16));
        g.Ellipse((30.5, 13.0, 36.0, 18.5), null, Ink, 1.1);
    }

    static void DrawSimpleTrail(IconCanvas g)
    {
        DrawRobot(g, 2, 12, 0.68);
        var trail = new[] { P(30, 24), P(34, 20), P(38, 23), P(43, 18) };
        g.Line(Shadow, 0.9, Offset(trail, 0.45, 0.45));
        g.Line(Orange, 0.8, trail);
    }

    static void DrawFloppy(IconCanvas g)
    {
        g.Polygon(Shadow, points: Points(9, 7, 34, 7, 40, 14, 40, 40, 9, 40));
        g.Polygon(Color(81, 93, 99), Ink, 1.6, Points(8, 6, 34, 6, 41, 13, 41, 41, 8, 41));
        g.Rect((13, 9, 32, 21), MetalLight, Ink, 1.0);
        g.Rect((28, 10, 33, 20), Color(42, 48, 51));
        g.Rounded((14, 28, 35, 38), 1.5, Paper, Ink, 1.0);
        g.Line(Orange, 1.6, P(17, 31), P(32, 31));
        g.Line(MetalDark, 1.2, P(17, 35), P(29, 35));
    }

    static void DrawWaitDi(IconCanvas g)
    {
        DrawClock(g, 30, 18, 11);
        g.Line(Ink, 1.4, P(8, 38), P(8, 24));
        DrawWave(g, Points(8, 34, 8, 28, 18, 28, 18, 34, 30, 34, 30, 28, 40, 28), Orange, 2.3, shadowWidth: 4.0);
    }

    static void DrawMessage(IconCanvas g)
    {
        using SKPath bubble = new();
        bubble.MoveTo(13, 9);
        bubble.LineTo(35, 9);
        bubble.QuadTo(40, 9, 40, 14);
        bubble.LineTo(40, 26);
        bubble.QuadTo(40, 31, 35, 31);
        bubble.LineTo(25, 31);
        bubble.LineTo(10, 40);
        bubble.LineTo(14, 31);
        bubble.LineTo(13, 31);
        bubble.QuadTo(8, 31, 8, 26);
        bubble.LineTo(8, 14);
        bubble.QuadTo(8, 9, 13, 9);
        bubble.Close();

        using SKPath shadow = new();
        shadow.AddPath(bubble, 1.1f, 1.1f);
        g.DrawPath(shadow, Shadow);
        g.DrawPath(bubble, Paper, Ink, 1.5);

        foreach (var x in new[] { 18.0, 24.0, 30.0 })
        {
            g.Ellipse((x - 2.1, 18.7, x + 2.1, 22.9), Orange, Ink, 0.65);
        }
    }

    static void DrawClock(IconCanvas g, double cx = 24, double cy = 24, double radius = 15)
    {
        g.Ellipse((cx - radius + 1.0, cy - radius + 1.0, cx + radius + 1.0, cy + radius + 1.0), Shadow);
        g.Ellipse((cx - radius, cy - radius, cx + radius, cy + radius), Paper, Ink, 1.8);
        g.Arc((cx - radius, cy - radius, cx + radius, cy + radius), 190, 350, Orange, 2.5);
        g.Line(Ink, 1.8, P(cx, cy), P(cx, cy - 8));
        g.Line(Ink, 1.8, P(cx, cy), P(cx + 7, cy + 5));

        foreach (var (dx, dy) in new[] { P(0, -11), P(11, 0), P(0, 11), P(-11, 0) })
        {
            g.Line(InkSoft, 1.1, P(cx + dx * 0.86, cy + dy * 0.86), P(cx + dx, cy + dy));
        }
    }

    static void DrawWave(IconCanvas g, (double X, double Y)[] points, SKColor color, double width = 3.0, double? shadowWidth = null)
    {
        g.Line(Shadow, shadowWidth ?? width, Offset(points, 0.9, 0.9));
        g.Line(Ink, width + 1.7, points);
        g.Line(color, width, points);
    }

    static void DrawStop(IconCanvas g)
    {
        using SKPath hand = new();
        hand.MoveTo(12, 28);
        hand.CubicTo(10, 25, 7, 25, 7, 28);
        hand.LineTo(14, 39);
        hand.CubicTo(16, 42, 20, 43, 25, 43);
        hand.CubicTo(32, 43, 36, 39, 36, 32);
        hand.LineTo(36, 20);
        hand.CubicTo(36, 17.5f, 32, 17.5f, 32, 20);
        hand.LineTo(32, 30);
        hand.LineTo(31, 30);
        hand.LineTo(31, 14);
        hand.CubicTo(31, 11.5f, 27, 11.5f, 27, 14);
        hand.LineTo(27, 30);
        hand.LineTo(26, 30);
        hand.LineTo(26, 10);
        hand.CubicTo(26, 7.5f, 22, 7.5f, 22, 10);
        hand.LineTo(22, 30);
        hand.LineTo(21, 30);
        hand.LineTo(21, 13);
        hand.CubicTo(21, 10.5f, 17, 10.5f, 17, 13);
        hand.LineTo(17, 33);
        hand.Close();

        using SKPath shadow = new();
        shadow.AddPath(hand, 1.1f, 1.1f);
        g.DrawPath(shadow, Shadow);
        g.DrawPath(hand, Orange, Ink, 1.2);
        g.Line(OrangeDark, 0.85, P(21.5, 14), P(21.5, 32));
        g.Line(OrangeDark, 0.85, P(26.5, 12), P(26.5, 32));
        g.Line(OrangeDark, 0.85, P(31.5, 21), P(31.5, 32));
    }

}
