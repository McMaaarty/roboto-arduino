using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Windows.Forms;

namespace RobotMapper;

internal sealed class DistancePlotControl : Control
{
    private readonly List<(DateTime t, int distMm)> _samples = new(capacity: 512);

    private double? _smoothedMm;
    private int? _lastOutputMm;

    public TimeSpan FenetreTemps { get; set; } = TimeSpan.FromSeconds(20);

    // Lissage d'affichage : réduit le jitter des capteurs ultrason (robot immobile).
    // Ces paramètres n'affectent QUE le graphe (pas la logique robot).
    public bool LissageActif { get; set; } = true;
    public double LissageAlpha { get; set; } = 0.18; // 0..1 (plus petit = plus stable)
    public int DeadbandMm { get; set; } = 3; // ignore les variations minimes
    public int MaxStepMm { get; set; } = 60; // limite le saut entre 2 points affichés

    public DistancePlotControl()
    {
        SetStyle(ControlStyles.AllPaintingInWmPaint
                 | ControlStyles.OptimizedDoubleBuffer
                 | ControlStyles.UserPaint
                 | ControlStyles.ResizeRedraw, true);

        BackColor = Color.FromArgb(12, 12, 12);
        ForeColor = Color.Gainsboro;
    }

    public void Reset()
    {
        _samples.Clear();
        _smoothedMm = null;
        _lastOutputMm = null;
        Invalidate();
    }

    public void AjouterPoint(DateTime horodatageLocal, int distanceMm)
    {
        var d = distanceMm;

        if (LissageActif)
            d = Lisser(distanceMm);

        _samples.Add((horodatageLocal, d));

        // Garde seulement la fenêtre (avec un peu de marge).
        var cutoff = horodatageLocal - FenetreTemps - TimeSpan.FromSeconds(1);
        int idx = 0;
        while (idx < _samples.Count && _samples[idx].t < cutoff)
            idx++;
        if (idx > 0)
            _samples.RemoveRange(0, idx);

        // Évite de grandir sans limite si le rafraîchissement UI est très fréquent.
        if (_samples.Count > 2500)
            _samples.RemoveRange(0, _samples.Count - 2500);

        Invalidate();
    }

    private int Lisser(int distanceMm)
    {
        // Si la distance est 0 (perte capteur), on affiche 0 et on reset le lissage.
        if (distanceMm <= 0)
        {
            _smoothedMm = null;
            _lastOutputMm = 0;
            return 0;
        }

        var alpha = LissageAlpha;
        if (alpha < 0.01) alpha = 0.01;
        if (alpha > 1.0) alpha = 1.0;

        _smoothedMm = _smoothedMm is null
            ? distanceMm
            : _smoothedMm.Value + alpha * (distanceMm - _smoothedMm.Value);

        var outMm = (int)Math.Round(_smoothedMm.Value);

        if (_lastOutputMm is int last)
        {
            var diff = outMm - last;
            if (Math.Abs(diff) <= DeadbandMm)
                return last;

            if (Math.Abs(diff) > MaxStepMm)
                outMm = last + Math.Sign(diff) * MaxStepMm;
        }

        _lastOutputMm = outMm;
        return outMm;
    }

    protected override void OnPaint(PaintEventArgs e)
    {
        base.OnPaint(e);

        e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
        e.Graphics.Clear(BackColor);

        var rect = ClientRectangle;
        if (rect.Width < 30 || rect.Height < 30)
            return;

        const int margeG = 42;
        const int margeD = 10;
        const int margeH = 10;
        const int margeB = 22;

        var plot = Rectangle.FromLTRB(
            rect.Left + margeG,
            rect.Top + margeH,
            rect.Right - margeD,
            rect.Bottom - margeB);

        using var penAxes = new Pen(Color.FromArgb(70, 70, 70), 1f);
        using var penGrid = new Pen(Color.FromArgb(35, 35, 35), 1f);
        using var penLine = new Pen(Color.FromArgb(0, 140, 0), 2f);

        // Axes
        e.Graphics.DrawLine(penAxes, plot.Left, plot.Bottom, plot.Right, plot.Bottom);
        e.Graphics.DrawLine(penAxes, plot.Left, plot.Top, plot.Left, plot.Bottom);

        if (_samples.Count < 2)
        {
            DessinerLabels(e.Graphics, plot, yMax: 0);
            return;
        }

        var tEnd = _samples[^1].t;
        var tStart = tEnd - FenetreTemps;
        var view = _samples.Where(s => s.t >= tStart && s.t <= tEnd).ToArray();
        if (view.Length < 2)
        {
            DessinerLabels(e.Graphics, plot, yMax: 0);
            return;
        }

        // Y scale
        int yMax = Math.Max(1, view.Max(s => Math.Max(0, s.distMm)));
        yMax = (int)Math.Ceiling(yMax * 1.1); // petite marge visuelle
        if (yMax < 200) yMax = 200;

        // Petite grille (2 lignes horizontales)
        for (int i = 1; i <= 2; i++)
        {
            int y = plot.Top + (plot.Height * i) / 3;
            e.Graphics.DrawLine(penGrid, plot.Left, y, plot.Right, y);
        }

        // Courbe
        var pts = new PointF[view.Length];
        double totalMs = Math.Max(1.0, (tEnd - tStart).TotalMilliseconds);

        for (int i = 0; i < view.Length; i++)
        {
            var s = view[i];
            double x01 = (s.t - tStart).TotalMilliseconds / totalMs;
            if (x01 < 0) x01 = 0;
            if (x01 > 1) x01 = 1;

            int d = Math.Max(0, s.distMm);
            double y01 = d / (double)yMax;
            if (y01 < 0) y01 = 0;
            if (y01 > 1) y01 = 1;

            float x = plot.Left + (float)(x01 * plot.Width);
            float y = plot.Bottom - (float)(y01 * plot.Height);
            pts[i] = new PointF(x, y);
        }

        e.Graphics.DrawLines(penLine, pts);

        // Dernier point
        using var brushPoint = new SolidBrush(Color.Gainsboro);
        e.Graphics.FillEllipse(brushPoint, pts[^1].X - 3, pts[^1].Y - 3, 6, 6);

        DessinerLabels(e.Graphics, plot, yMax);
    }

    private void DessinerLabels(Graphics g, Rectangle plot, int yMax)
    {
        using var brush = new SolidBrush(ForeColor);
        var famille = SystemFonts.MessageBoxFont?.FontFamily ?? FontFamily.GenericSansSerif;
        using var font = new Font(famille, 8.25f, FontStyle.Regular);

        // Axe Y
        g.DrawString("mm", font, brush, new PointF(plot.Left - 30, plot.Top - 2));
        if (yMax > 0)
            g.DrawString(yMax.ToString(), font, brush, new PointF(plot.Left - 38, plot.Top + 10));
        g.DrawString("0", font, brush, new PointF(plot.Left - 18, plot.Bottom - 12));

        // Axe X
        g.DrawString($"t (≈{(int)FenetreTemps.TotalSeconds}s)", font, brush, new PointF(plot.Right - 70, plot.Bottom + 4));
    }
}
