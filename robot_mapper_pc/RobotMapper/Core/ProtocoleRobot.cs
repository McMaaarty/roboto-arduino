using System;
using System.Globalization;

namespace RobotMapper.Core;

/// <summary>
/// Parse les trames ASCII en provenance du robot.
/// </summary>
internal static class ProtocoleRobot
{
    public static bool TryParseTelemetrie(string ligne, out Telemetrie telemetrie)
    {
        telemetrie = default;
        if (!ligne.StartsWith("T,", StringComparison.Ordinal))
            return false;

        // T,<ms>,<x_mm>,<y_mm>,<yaw_cdeg>,<dist_mm>,<mode>
        var parts = ligne.Split(',');
        if (parts.Length < 7)
            return false;

        if (!uint.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var ms)) return false;
        if (!int.TryParse(parts[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out var xMm)) return false;
        if (!int.TryParse(parts[3], NumberStyles.Integer, CultureInfo.InvariantCulture, out var yMm)) return false;
        if (!int.TryParse(parts[4], NumberStyles.Integer, CultureInfo.InvariantCulture, out var yawCdeg)) return false;
        if (!int.TryParse(parts[5], NumberStyles.Integer, CultureInfo.InvariantCulture, out var distMm)) return false;
        if (!int.TryParse(parts[6], NumberStyles.Integer, CultureInfo.InvariantCulture, out var mode)) return false;

        var motion = '\0';
        if (parts.Length >= 8 && parts[7].Length > 0)
            motion = parts[7][0];

        var hasAccel = false;
        var axMg = 0;
        var ayMg = 0;
        var azMg = 0;
        if (parts.Length >= 11
            && int.TryParse(parts[8], NumberStyles.Integer, CultureInfo.InvariantCulture, out axMg)
            && int.TryParse(parts[9], NumberStyles.Integer, CultureInfo.InvariantCulture, out ayMg)
            && int.TryParse(parts[10], NumberStyles.Integer, CultureInfo.InvariantCulture, out azMg))
        {
            hasAccel = true;
        }

        var hasEncoders = false;
        var encLeft = 0;
        var encRight = 0;
        if (parts.Length >= 13
            && int.TryParse(parts[11], NumberStyles.Integer, CultureInfo.InvariantCulture, out encLeft)
            && int.TryParse(parts[12], NumberStyles.Integer, CultureInfo.InvariantCulture, out encRight))
        {
            hasEncoders = true;
        }

        telemetrie = new Telemetrie(ms, xMm, yMm, yawCdeg, distMm, mode, motion, hasAccel, axMg, ayMg, azMg, hasEncoders, encLeft, encRight);
        return true;
    }

    public static bool TryParseDistance(string ligne, out int distanceMm)
    {
        distanceMm = 0;
        if (!ligne.StartsWith("D,", StringComparison.Ordinal))
            return false;

        var parts = ligne.Split(',');
        return parts.Length >= 2
            && int.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out distanceMm);
    }
}

internal readonly record struct Telemetrie(
    uint Ms,
    int XMm,
    int YMm,
    int YawCdeg,
    int DistMm,
    int Mode,
    char Motion,
    bool HasAccel,
    int AxMg,
    int AyMg,
    int AzMg,
    bool HasEncoders,
    int EncLeft,
    int EncRight);
