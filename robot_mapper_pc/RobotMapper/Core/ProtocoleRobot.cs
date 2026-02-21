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

        if (!int.TryParse(parts[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out var xMm)) return false;
        if (!int.TryParse(parts[3], NumberStyles.Integer, CultureInfo.InvariantCulture, out var yMm)) return false;
        if (!int.TryParse(parts[4], NumberStyles.Integer, CultureInfo.InvariantCulture, out var yawCdeg)) return false;
        if (!int.TryParse(parts[5], NumberStyles.Integer, CultureInfo.InvariantCulture, out var distMm)) return false;
        if (!int.TryParse(parts[6], NumberStyles.Integer, CultureInfo.InvariantCulture, out var mode)) return false;

        telemetrie = new Telemetrie(xMm, yMm, yawCdeg, distMm, mode);
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

internal readonly record struct Telemetrie(int XMm, int YMm, int YawCdeg, int DistMm, int Mode);
