using System;
using System.Collections.Generic;

namespace RobotMapper.Core;

/// <summary>
/// Logique principale : état robot + parsing des trames + mise à jour de la carte.
/// </summary>
internal sealed class RobotMapperCore
{
    public EtatRobot Etat { get; } = new();
    public CarteOccupation Carte { get; } = new();

    /// <summary>
    /// Traite une ligne reçue du robot.
    /// </summary>
    /// <returns>Liste des cellules modifiées dans la carte (peut être vide).</returns>
    public List<CelluleChangee> TraiterLigne(string ligne)
    {
        if (ProtocoleRobot.TryParseTelemetrie(ligne, out var t))
        {
            Etat.PositionXMm = t.XMm;
            Etat.PositionYMm = t.YMm;
            Etat.CapCdeg = t.YawCdeg;
            Etat.DistanceMm = t.DistMm;
            Etat.Mode = t.Mode;
            Etat.DerniereTelemetrieA = DateTime.Now;

            if (t.DistMm > 0)
                return Carte.MettreAJourAvecMesure(t.XMm, t.YMm, t.YawCdeg, t.DistMm);

            return new List<CelluleChangee>(0);
        }

        if (ProtocoleRobot.TryParseDistance(ligne, out var distance))
        {
            Etat.DistanceMm = distance;
            return new List<CelluleChangee>(0);
        }

        return new List<CelluleChangee>(0);
    }
}
