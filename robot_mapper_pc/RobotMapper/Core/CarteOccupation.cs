using System;
using System.Collections.Generic;

namespace RobotMapper.Core;

/// <summary>
/// Carte d'occupation sous forme de grille (inconnu/libre/occupé) mise à jour par ray tracing.
/// </summary>
internal sealed class CarteOccupation
{
    public const int Largeur = 201;
    public const int Hauteur = 201;
    public const int TailleCelluleMm = 50; // 5cm

    private readonly sbyte[,] _grille = new sbyte[Largeur, Hauteur];

    public CarteOccupation()
    {
        for (int x = 0; x < Largeur; x++)
        for (int y = 0; y < Hauteur; y++)
            _grille[x, y] = -1;
    }

    public sbyte[,] Grille => _grille;

    public (int gx, int gy) MondeVersGrille(int xMm, int yMm)
    {
        var cx = Largeur / 2;
        var cy = Hauteur / 2;
        var gx = cx + (int)Math.Round(xMm / (double)TailleCelluleMm);
        var gy = cy - (int)Math.Round(yMm / (double)TailleCelluleMm);
        gx = Clamp(gx, 0, Largeur - 1);
        gy = Clamp(gy, 0, Hauteur - 1);
        return (gx, gy);
    }

    public (int xMm, int yMm) GrilleVersMonde(int gx, int gy)
    {
        var cx = Largeur / 2;
        var cy = Hauteur / 2;
        var xMm = (gx - cx) * TailleCelluleMm;
        var yMm = (cy - gy) * TailleCelluleMm;
        return (xMm, yMm);
    }

    public List<CelluleChangee> MettreAJourAvecMesure(int xMm, int yMm, int yawCdeg, int distMm)
    {
        var changements = new List<CelluleChangee>(256);

        var yawRad = (yawCdeg / 100.0) * (Math.PI / 180.0);
        var ex = xMm + (int)(distMm * Math.Cos(yawRad));
        var ey = yMm + (int)(distMm * Math.Sin(yawRad));

        var (sx, sy) = MondeVersGrille(xMm, yMm);
        var (gx, gy) = MondeVersGrille(ex, ey);

        TracerRayon(sx, sy, gx, gy, changements);
        return changements;
    }

    private static int Clamp(int v, int min, int max) => v < min ? min : (v > max ? max : v);

    private void DefinirCellule(int gx, int gy, sbyte valeur, List<CelluleChangee> changements)
    {
        if (gx < 0 || gy < 0 || gx >= Largeur || gy >= Hauteur)
            return;

        // Ne pas effacer un obstacle par "libre".
        if (valeur == 0 && _grille[gx, gy] == 1)
            return;

        if (_grille[gx, gy] == valeur)
            return;

        _grille[gx, gy] = valeur;
        changements.Add(new CelluleChangee(gx, gy, valeur));
    }

    private void TracerRayon(int x0, int y0, int x1, int y1, List<CelluleChangee> changements)
    {
        // Bresenham
        int dx = Math.Abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -Math.Abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        int x = x0;
        int y = y0;

        // Marque libre le long du rayon, occupé à la fin.
        while (true)
        {
            if (x == x1 && y == y1)
                break;

            DefinirCellule(x, y, 0, changements);

            int e2 = 2 * err;
            if (e2 >= dy)
            {
                err += dy;
                x += sx;
            }
            if (e2 <= dx)
            {
                err += dx;
                y += sy;
            }

            if (x < 0 || y < 0 || x >= Largeur || y >= Hauteur)
                break;
        }

        DefinirCellule(x1, y1, 1, changements);
    }
}

internal readonly record struct CelluleChangee(int Gx, int Gy, sbyte Valeur);
