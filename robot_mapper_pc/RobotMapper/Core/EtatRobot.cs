using System;

namespace RobotMapper.Core;

/// <summary>
/// État courant du robot (dernières valeurs connues).
/// </summary>
internal sealed class EtatRobot
{
    public int PositionXMm { get; set; }
    public int PositionYMm { get; set; }
    public int CapCdeg { get; set; }
    public int DistanceMm { get; set; }
    public int Mode { get; set; }

    public DateTime DerniereTelemetrieA { get; set; } = DateTime.MinValue;
}
