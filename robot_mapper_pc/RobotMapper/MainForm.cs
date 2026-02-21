using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RobotMapper;

/// <summary>
/// Fenêtre principale WinForms de RobotMapper.
/// 
/// Rôle :
/// - Se connecter à un port série (Bluetooth SPP / USB).
/// - Recevoir la télémétrie et mettre à jour une grille d'occupation.
/// - Envoyer des commandes simples au robot (mouvement, ping, mode, objectif).
/// </summary>
public sealed class MainForm : Form
{
    private const int VitesseBaud = 9600;
    private const int TimeoutLectureMs = 500;
    private const int TimeoutEcritureMs = 500;
    private const int IntervalleRafraichissementUiMs = 50;
    private const int LignesLogMax = 650;
    private const int LignesLogConservees = 600;

    private readonly ComboBox _comboPortsCom = new() { DropDownStyle = ComboBoxStyle.DropDownList, Width = 140 };
    private readonly Button _boutonRafraichir = new() { Text = "Refresh", Width = 80 };
    private readonly Button _boutonConnecter = new() { Text = "Connect", Width = 80 };
    private readonly Button _boutonDeconnecter = new() { Text = "Disconnect", Width = 80, Enabled = false };
    private readonly Button _boutonStream = new() { Text = "Stream (M)", Width = 100, Enabled = false };
    private readonly Button _boutonAuto = new() { Text = "Auto (A)", Width = 90, Enabled = false };
    private readonly Button _boutonPing = new() { Text = "Ping (P)", Width = 90, Enabled = false };
    private readonly Label _labelStatut = new() { AutoSize = true, Text = "Disconnected" };
    private readonly DoubleBufferedPanel _panneauCarte = new() { Dock = DockStyle.Fill, BackColor = Color.Black };
    private readonly TextBox _zoneLog = new()
    {
        Dock = DockStyle.Bottom,
        Height = 180,
        Multiline = true,
        ReadOnly = true,
        ScrollBars = ScrollBars.Vertical,
        Font = new Font("Consolas", 9f),
    };

    private SerialPort? _portSerie;
    private string? _nomPortConnecte;

    private readonly object _verrouReception = new();
    private readonly StringBuilder _tamponReception = new();

    // Carte d'occupation (grille)
    private const int GridW = 201;
    private const int GridH = 201;
    private const int CellSizeMm = 50; // 5cm
    private readonly sbyte[,] _grille = new sbyte[GridW, GridH]; // -1 inconnu, 0 libre, 1 occupé
    private readonly Bitmap _bitmapGrille = new(GridW, GridH);

    // État télémétrie (dernières valeurs reçues)
    private int _positionXMm;
    private int _positionYMm;
    private int _capCdeg;
    private int _distanceMm;
    private int _mode;
    private bool _objectifDefini;
    private int _objectifXMm;
    private int _objectifYMm;

    private int _lignesRx;
    private int _lignesTx;
    private bool _rafraichissementEnAttente;

    private DateTime _derniereTelemetrieA = DateTime.MinValue;
    private string _infoDerniereTouche = "";

    private readonly HashSet<Keys> _touchesEnfoncees = new();

    private readonly System.Windows.Forms.Timer _timerRafraichissement = new() { Interval = IntervalleRafraichissementUiMs };

    // UI tableau de bord (télémétrie + feedback)
    private readonly StatusStrip _barreStatut = new() { SizingGrip = false };
    private readonly ToolStripStatusLabel _statutPrincipal = new() { Spring = true, TextAlign = ContentAlignment.MiddleLeft };
    private readonly ToolStripStatusLabel _statutSecondaire = new() { TextAlign = ContentAlignment.MiddleRight };

    private readonly ListBox _listeActions = new() { Dock = DockStyle.Top, Height = 92 };
    private readonly Queue<string> _historiqueActions = new();

    private readonly Panel _panneauTableauDeBord = new() { Dock = DockStyle.Fill };
    private readonly Panel _voyantConnexion = new() { Width = 12, Height = 12 };
    private readonly Panel _voyantTelemetrie = new() { Width = 12, Height = 12 };
    private readonly Label _valPort = new() { AutoSize = true };
    private readonly Label _valConnexion = new() { AutoSize = true };
    private readonly Label _valX = new() { AutoSize = true };
    private readonly Label _valY = new() { AutoSize = true };
    private readonly Label _valYaw = new() { AutoSize = true };
    private readonly Label _valDistance = new() { AutoSize = true };
    private readonly Label _valMode = new() { AutoSize = true };
    private readonly Label _valAgeTelemetrie = new() { AutoSize = true };
    private readonly Label _valRxTx = new() { AutoSize = true };

    private bool EstConnecte => _portSerie is not null && _portSerie.IsOpen;

    public MainForm()
    {
        Text = "Robot Mapper";
        Width = 1100;
        Height = 800;

        KeyPreview = true;
        DoubleBuffered = true;

        // Initialisation de la carte : tout est inconnu au départ.
        for (int x = 0; x < GridW; x++)
        for (int y = 0; y < GridH; y++)
            _grille[x, y] = -1;

        InitialiserBitmap();

        // Style “cockpit” (sobre) : fond sombre + contrôles plats.
        BackColor = Color.FromArgb(18, 18, 18);
        ForeColor = Color.Gainsboro;

        AppliquerStyleBouton(_boutonRafraichir);
        AppliquerStyleBouton(_boutonConnecter);
        AppliquerStyleBouton(_boutonDeconnecter);
        AppliquerStyleBouton(_boutonStream);
        AppliquerStyleBouton(_boutonAuto);
        AppliquerStyleBouton(_boutonPing);

        _comboPortsCom.BackColor = Color.FromArgb(30, 30, 30);
        _comboPortsCom.ForeColor = Color.Gainsboro;

        _zoneLog.BackColor = Color.FromArgb(12, 12, 12);
        _zoneLog.ForeColor = Color.Gainsboro;
        _zoneLog.BorderStyle = BorderStyle.FixedSingle;

        _listeActions.BackColor = Color.FromArgb(12, 12, 12);
        _listeActions.ForeColor = Color.Gainsboro;
        _listeActions.BorderStyle = BorderStyle.FixedSingle;

        // Barre d'outils (haut)
        var barreHaut = new FlowLayoutPanel
        {
            Dock = DockStyle.Fill,
            Height = 46,
            Padding = new Padding(10, 8, 10, 8),
            WrapContents = false,
            BackColor = Color.FromArgb(24, 24, 24),
        };
        barreHaut.Controls.Add(new Label
        {
            Text = "COM",
            AutoSize = true,
            Padding = new Padding(0, 7, 6, 0),
            ForeColor = Color.Gainsboro
        });
        barreHaut.Controls.Add(_comboPortsCom);
        barreHaut.Controls.Add(_boutonRafraichir);
        barreHaut.Controls.Add(_boutonConnecter);
        barreHaut.Controls.Add(_boutonDeconnecter);
        barreHaut.Controls.Add(Espace(10));
        barreHaut.Controls.Add(_boutonStream);
        barreHaut.Controls.Add(_boutonAuto);
        barreHaut.Controls.Add(_boutonPing);
        barreHaut.Controls.Add(Espace(12));
        _labelStatut.ForeColor = Color.Silver;
        barreHaut.Controls.Add(_labelStatut);

        // Contenu central : carte (gauche) + tableau de bord (droite)
        ConstruireTableauDeBord();
        var splitCentre = new SplitContainer
        {
            Dock = DockStyle.Fill,
            Orientation = Orientation.Vertical,
            SplitterWidth = 6,
            BackColor = Color.FromArgb(18, 18, 18),
            FixedPanel = FixedPanel.Panel2,
        };
        splitCentre.Panel1.Controls.Add(_panneauCarte);
        splitCentre.Panel2.Controls.Add(_panneauTableauDeBord);

        // IMPORTANT : ne pas configurer min sizes / SplitterDistance dans le constructeur.
        // Le SplitContainer n'a pas encore une taille stable (layout/DPI), ce qui peut provoquer
        // l'exception "SplitterDistance doit se situer entre ...".
        splitCentre.HandleCreated += (_, _) => ConfigurerSplitCentre(splitCentre, largeurTableauDeBordSouhaitee: 340);
        splitCentre.SizeChanged += (_, _) => ConfigurerSplitCentre(splitCentre, largeurTableauDeBordSouhaitee: 340);

        // Bas : historique actions (compact) + log
        var bas = new Panel { Dock = DockStyle.Fill, BackColor = Color.FromArgb(18, 18, 18) };
        bas.Controls.Add(_zoneLog);
        bas.Controls.Add(_listeActions);
        _zoneLog.Dock = DockStyle.Fill;

        // Barre de statut (tout en bas)
        _barreStatut.BackColor = Color.FromArgb(24, 24, 24);
        _barreStatut.ForeColor = Color.Gainsboro;
        _barreStatut.Items.Add(_statutPrincipal);
        _barreStatut.Items.Add(_statutSecondaire);

        // Layout global
        var layout = new TableLayoutPanel
        {
            Dock = DockStyle.Fill,
            ColumnCount = 1,
            RowCount = 4,
            BackColor = Color.FromArgb(18, 18, 18),
        };
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 46));
        layout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 240));
        layout.RowStyles.Add(new RowStyle(SizeType.Absolute, 24));
        layout.Controls.Add(barreHaut, 0, 0);
        layout.Controls.Add(splitCentre, 0, 1);
        layout.Controls.Add(bas, 0, 2);
        layout.Controls.Add(_barreStatut, 0, 3);
        Controls.Add(layout);

        _panneauCarte.Paint += PanneauCarteOnPaint;
        _panneauCarte.MouseClick += PanneauCarteOnMouseClick;

        KeyDown += MainFormOnKeyDown;
        KeyUp += MainFormOnKeyUp;

        _boutonRafraichir.Click += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                FlashBouton(_boutonRafraichir);
                RafraichirListePorts();
                NotifierAction("Ports COM rafraîchis");
            }, "Rafraîchissement des ports COM", this);

        _boutonConnecter.Click += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                FlashBouton(_boutonConnecter);
                Connecter();
            }, "Connexion port série", this);

        _boutonDeconnecter.Click += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                FlashBouton(_boutonDeconnecter);
                Deconnecter();
                NotifierAction("Déconnecté");
            }, "Déconnexion port série", this, afficherDialogue: false);

        _boutonStream.Click += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                FlashBouton(_boutonStream);
                EnvoyerLigne("M\n");
                NotifierAction("Commande envoyée : M (stream)");
            }, "Commande Stream (M)", this, afficherDialogue: false);

        _boutonAuto.Click += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                FlashBouton(_boutonAuto);
                EnvoyerLigne("A\n");
                NotifierAction("Commande envoyée : A (auto)");
            }, "Commande Auto (A)", this, afficherDialogue: false);

        _boutonPing.Click += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                FlashBouton(_boutonPing);
                EnvoyerLigne("P\n");
                NotifierAction("Commande envoyée : P (ping)");
            }, "Commande Ping (P)", this, afficherDialogue: false);

        _timerRafraichissement.Tick += (_, _) =>
            GestionErreurs.Executer(() =>
            {
                MettreAJourTexteStatut();
                MettreAJourTableauDeBord();
                if (_rafraichissementEnAttente)
                {
                    _rafraichissementEnAttente = false;
                    _panneauCarte.Invalidate();
                }
            }, "Tick rafraîchissement UI", this, afficherDialogue: false);
        _timerRafraichissement.Start();

        RafraichirListePorts();
        MettreAJourTableauDeBord();
        NotifierAction("Prêt");
    }

    private static Control Espace(int largeur) => new Panel { Width = largeur, Height = 1 };

    private static void ConfigurerSplitCentre(SplitContainer split, int largeurTableauDeBordSouhaitee)
    {
        if (split.Orientation != Orientation.Vertical)
            return;

        var largeurTotale = split.Width;
        if (largeurTotale <= 0)
            return;

        // Objectif : éviter toute valeur invalide même si la fenêtre est petite / DPI scaling.
        // On calcule des min sizes compatibles avec la largeur disponible.
        const int panel1MinSouhaite = 420;
        const int panel2MinSouhaite = 320;
        const int minAbsolu = 140;

        var largeurDisponible = Math.Max(0, largeurTotale - split.SplitterWidth);

        var panel2Min = panel2MinSouhaite;
        var panel1Min = panel1MinSouhaite;

        if (panel1Min + panel2Min > largeurDisponible)
        {
            // Réduction simple : on garantit un minimum absolu, puis on donne le reste à Panel1.
            panel2Min = Math.Max(minAbsolu, Math.Min(panel2MinSouhaite, largeurDisponible / 3));
            panel1Min = Math.Max(minAbsolu, largeurDisponible - panel2Min);
        }

        // Calcule une SplitterDistance cible (Panel2 ~ largeur souhaitée) en respectant les min sizes.
        var minDistance = panel1Min;
        var maxDistance = largeurTotale - panel2Min - split.SplitterWidth;
        if (maxDistance < minDistance)
            return;

        var distanceVoulue = largeurTotale - largeurTableauDeBordSouhaitee - split.SplitterWidth;
        var distanceClampee = Math.Max(minDistance, Math.Min(maxDistance, distanceVoulue));

        // Astuce anti-exception : on pose d'abord une distance valide (min sizes actuels sont encore permissifs),
        // puis on fixe les min sizes. Ainsi, ApplyPanel2MinSize n'a pas besoin de corriger vers une valeur invalide.
        if (split.SplitterDistance < 0 || split.SplitterDistance > largeurTotale)
            split.SplitterDistance = distanceClampee;
        else if (Math.Abs(split.SplitterDistance - distanceClampee) > 40)
            split.SplitterDistance = distanceClampee;

        if (split.Panel1MinSize != panel1Min)
            split.Panel1MinSize = panel1Min;
        if (split.Panel2MinSize != panel2Min)
            split.Panel2MinSize = panel2Min;
    }

    private static void AppliquerStyleBouton(Button bouton)
    {
        bouton.FlatStyle = FlatStyle.Flat;
        bouton.FlatAppearance.BorderSize = 1;
        bouton.FlatAppearance.BorderColor = Color.FromArgb(70, 70, 70);
        bouton.BackColor = Color.FromArgb(32, 32, 32);
        bouton.ForeColor = Color.Gainsboro;
        bouton.Height = 28;
    }

    private async void FlashBouton(Control bouton)
    {
        var couleurInitiale = bouton.BackColor;
        bouton.BackColor = Color.FromArgb(60, 60, 60);
        await Task.Delay(120);
        if (!IsDisposed)
            bouton.BackColor = couleurInitiale;
    }

    private void NotifierAction(string message)
    {
        var horodatage = DateTime.Now.ToString("HH:mm:ss");
        var ligne = $"{horodatage}  {message}";

        _statutPrincipal.Text = message;
        _statutSecondaire.Text = EstConnecte ? $"{_nomPortConnecte} @{VitesseBaud}" : "Hors ligne";

        _historiqueActions.Enqueue(ligne);
        while (_historiqueActions.Count > 5)
            _historiqueActions.Dequeue();

        _listeActions.BeginUpdate();
        _listeActions.Items.Clear();
        foreach (var item in _historiqueActions.Reverse())
            _listeActions.Items.Add(item);
        _listeActions.EndUpdate();
    }

    private void ConstruireTableauDeBord()
    {
        _panneauTableauDeBord.BackColor = Color.FromArgb(22, 22, 22);
        _panneauTableauDeBord.Padding = new Padding(12);

        var titre = new Label
        {
            Text = "TABLEAU DE BORD",
            AutoSize = true,
            ForeColor = Color.Gainsboro,
            Font = new Font(Font.FontFamily, 10f, FontStyle.Bold),
            Dock = DockStyle.Top,
            Margin = new Padding(0, 0, 0, 10),
            Padding = new Padding(0, 0, 0, 6),
        };

        var grille = new TableLayoutPanel
        {
            Dock = DockStyle.Top,
            AutoSize = true,
            ColumnCount = 2,
            RowCount = 10,
            BackColor = Color.FromArgb(22, 22, 22),
        };
        grille.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 45));
        grille.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 55));

        StyleValeur(_valPort);
        StyleValeur(_valConnexion);
        StyleValeur(_valX);
        StyleValeur(_valY);
        StyleValeur(_valYaw);
        StyleValeur(_valDistance);
        StyleValeur(_valMode);
        StyleValeur(_valAgeTelemetrie);
        StyleValeur(_valRxTx);

        AjouterLigne(grille, 0, "Connexion", ConstruireLigneVoyants());
        AjouterLigne(grille, 1, "Port", _valPort);
        AjouterLigne(grille, 2, "État", _valConnexion);
        AjouterLigne(grille, 3, "X (mm)", _valX);
        AjouterLigne(grille, 4, "Y (mm)", _valY);
        AjouterLigne(grille, 5, "Yaw (°)", _valYaw);
        AjouterLigne(grille, 6, "Distance (mm)", _valDistance);
        AjouterLigne(grille, 7, "Mode", _valMode);
        AjouterLigne(grille, 8, "Âge télémétrie", _valAgeTelemetrie);
        AjouterLigne(grille, 9, "Compteurs", _valRxTx);

        _panneauTableauDeBord.Controls.Clear();
        _panneauTableauDeBord.Controls.Add(grille);
        _panneauTableauDeBord.Controls.Add(titre);
    }

    private Control ConstruireLigneVoyants()
    {
        var conteneur = new FlowLayoutPanel
        {
            AutoSize = true,
            WrapContents = false,
            BackColor = Color.FromArgb(22, 22, 22),
            Margin = new Padding(0)
        };

        InitialiserVoyant(_voyantConnexion);
        InitialiserVoyant(_voyantTelemetrie);

        conteneur.Controls.Add(_voyantConnexion);
        conteneur.Controls.Add(new Label { Text = "Connecté", AutoSize = true, ForeColor = Color.Silver, Padding = new Padding(6, 0, 14, 0) });
        conteneur.Controls.Add(_voyantTelemetrie);
        conteneur.Controls.Add(new Label { Text = "Télémétrie", AutoSize = true, ForeColor = Color.Silver, Padding = new Padding(6, 0, 0, 0) });

        return conteneur;
    }

    private static void InitialiserVoyant(Panel voyant)
    {
        voyant.BackColor = Color.FromArgb(90, 0, 0);
        voyant.Margin = new Padding(0, 4, 0, 0);
        voyant.Paint += (_, e) =>
        {
            e.Graphics.SmoothingMode = SmoothingMode.AntiAlias;
            using var brush = new SolidBrush(voyant.BackColor);
            e.Graphics.FillEllipse(brush, 0, 0, voyant.Width - 1, voyant.Height - 1);
        };
    }

    private static void AjouterLigne(TableLayoutPanel grille, int ligne, string libelle, Control valeur)
    {
        var etiquette = new Label
        {
            Text = libelle,
            AutoSize = true,
            ForeColor = Color.Silver,
            Padding = new Padding(0, 3, 0, 3),
            Margin = new Padding(0, 3, 0, 3)
        };
        valeur.Margin = new Padding(0, 3, 0, 3);
        grille.Controls.Add(etiquette, 0, ligne);
        grille.Controls.Add(valeur, 1, ligne);
    }

    private static void StyleValeur(Label label)
    {
        label.ForeColor = Color.Gainsboro;
        var famille = SystemFonts.MessageBoxFont?.FontFamily ?? label.Font.FontFamily;
        label.Font = new Font(famille, 9f, FontStyle.Bold);
    }

    private void MettreAJourTableauDeBord()
    {
        _valPort.Text = _nomPortConnecte ?? "—";
        _valConnexion.Text = EstConnecte ? "Connecté" : "Déconnecté";
        _valX.Text = _positionXMm.ToString(CultureInfo.InvariantCulture);
        _valY.Text = _positionYMm.ToString(CultureInfo.InvariantCulture);
        _valYaw.Text = (_capCdeg / 100.0).ToString("F1", CultureInfo.InvariantCulture);
        _valDistance.Text = _distanceMm.ToString(CultureInfo.InvariantCulture);
        _valMode.Text = $"{_mode} ({ModeToText(_mode)})";
        _valRxTx.Text = $"RX={_lignesRx}  TX={_lignesTx}";

        if (_derniereTelemetrieA == DateTime.MinValue)
        {
            _valAgeTelemetrie.Text = "—";
        }
        else
        {
            var age = DateTime.Now - _derniereTelemetrieA;
            _valAgeTelemetrie.Text = $"{age.TotalMilliseconds:0} ms";
        }

        // Voyant connexion
        _voyantConnexion.BackColor = EstConnecte ? Color.FromArgb(0, 140, 0) : Color.FromArgb(140, 0, 0);

        // Voyant télémétrie : vert si récent, orange si vieux, rouge si absent.
        if (!EstConnecte || _derniereTelemetrieA == DateTime.MinValue)
        {
            _voyantTelemetrie.BackColor = Color.FromArgb(140, 0, 0);
        }
        else
        {
            var ageMs = (DateTime.Now - _derniereTelemetrieA).TotalMilliseconds;
            _voyantTelemetrie.BackColor = ageMs < 700
                ? Color.FromArgb(0, 140, 0)
                : ageMs < 2000
                    ? Color.FromArgb(170, 110, 0)
                    : Color.FromArgb(140, 0, 0);
        }

        _voyantConnexion.Invalidate();
        _voyantTelemetrie.Invalidate();
    }

    private void Journaliser(string direction, string message)
    {
        // Journal minimaliste pour debug (RX/TX + événements UI).
        var line = $"{DateTime.Now:HH:mm:ss.fff} {direction} {message}";
        _zoneLog.AppendText(line + Environment.NewLine);

        // Limit size (keep last ~600 lines)
        var text = _zoneLog.Text;
        var lines = text.Split(new[] { "\r\n", "\n" }, StringSplitOptions.None);
        if (lines.Length > LignesLogMax)
        {
            var keep = string.Join(Environment.NewLine, lines.Skip(lines.Length - LignesLogConservees));
            _zoneLog.Text = keep;
            _zoneLog.SelectionStart = _zoneLog.TextLength;
            _zoneLog.ScrollToCaret();
        }
    }

    /// <summary>
    /// Active/désactive les boutons en fonction de l'état de connexion.
    /// Centralisé pour éviter les incohérences UI lors des évolutions.
    /// </summary>
    private void MettreAJourEtatConnexion(bool estConnecte)
    {
        _boutonConnecter.Enabled = !estConnecte;
        _boutonDeconnecter.Enabled = estConnecte;
        _boutonStream.Enabled = estConnecte;
        _boutonAuto.Enabled = estConnecte;
        _boutonPing.Enabled = estConnecte;
    }

    private void DemanderRafraichissementCarte()
    {
        _rafraichissementEnAttente = true;
    }

    private static string ModeToText(int mode)
    {
        return mode switch
        {
            0 => "MANUAL",
            1 => "AUTO_EXPLORE",
            2 => "AUTO_GOTO",
            _ => "UNKNOWN"
        };
    }

    private void MettreAJourTexteStatut()
    {
        if (!EstConnecte)
            return;

        var modeText = ModeToText(_mode);

        string ageText;
        if (_derniereTelemetrieA == DateTime.MinValue)
        {
            ageText = "No telemetry (click Stream (M))";
        }
        else
        {
            var age = DateTime.Now - _derniereTelemetrieA;
            ageText = $"Last T={age.TotalMilliseconds:0}ms ago";
        }

        _labelStatut.Text = $"{_nomPortConnecte} @{VitesseBaud} | x={_positionXMm} y={_positionYMm} yaw={_capCdeg / 100.0:F1}° dist={_distanceMm} mode={_mode}({modeText}) | {ageText} | RX={_lignesRx} TX={_lignesTx} {_infoDerniereTouche}";
    }

    private void InitialiserBitmap()
    {
        for (int x = 0; x < GridW; x++)
        for (int y = 0; y < GridH; y++)
            _bitmapGrille.SetPixel(x, y, Color.FromArgb(40, 40, 40));
    }

    /// <summary>
    /// Recharge la liste des ports COM disponibles et sélectionne le premier si présent.
    /// </summary>
    private void RafraichirListePorts()
    {
        var ports = SerialPort.GetPortNames().OrderBy(p => p).ToArray();
        _comboPortsCom.Items.Clear();
        _comboPortsCom.Items.AddRange(ports);
        if (ports.Length > 0)
            _comboPortsCom.SelectedIndex = 0;
    }

    /// <summary>
    /// Ouvre le port série sélectionné et installe le handler de réception.
    /// </summary>
    private void Connecter()
    {
        if (_comboPortsCom.SelectedItem is not string portName)
            return;

        Deconnecter();

        _portSerie = new SerialPort(portName, VitesseBaud)
        {
            NewLine = "\n",
            Encoding = Encoding.ASCII,
            ReadTimeout = TimeoutLectureMs,
            WriteTimeout = TimeoutEcritureMs
        };

        _portSerie.DataReceived += (_, _) =>
        {
            try
            {
                if (_portSerie is null) return;

                var chunk = _portSerie.ReadExisting();
                if (string.IsNullOrEmpty(chunk))
                    return;

                List<string> lines = new();

                lock (_verrouReception)
                {
                    _tamponReception.Append(chunk);

                    while (true)
                    {
                        var s = _tamponReception.ToString();
                        var idx = s.IndexOf('\n');
                        if (idx < 0)
                            break;

                        var line = s.Substring(0, idx);
                        // drop consumed + '\n'
                        _tamponReception.Clear();
                        _tamponReception.Append(s.Substring(idx + 1));

                        line = line.Trim('\r');
                        if (line.Length > 0)
                            lines.Add(line);
                    }
                }

                if (lines.Count > 0)
                {
                    BeginInvoke(new Action(() =>
                    {
                        foreach (var line in lines)
                            TraiterLigneRecue(line);
                    }));
                }
            }
            catch (Exception ex)
            {
                // Ne pas faire crasher l'application sur une erreur de lecture (câble, BT instable, etc.).
                GestionErreurs.Signaler(ex, "Lecture port série (DataReceived)", this, afficherDialogue: false);
            }
        };

        try
        {
            _portSerie.Open();
        }
        catch (UnauthorizedAccessException)
        {
            _portSerie.Dispose();
            _portSerie = null;
            _labelStatut.Text = $"Cannot open {portName}";
            MessageBox.Show(
                this,
                "Accès refusé à " + portName + ".\n\n" +
                "Ca arrive le plus souvent quand :\n" +
                "- Le port est déjà ouvert par une autre appli (Arduino IDE / Serial Monitor / autre).\n" +
                "- Tu as choisi le COM Bluetooth \"Incoming\" au lieu du \"Outgoing\" (SPP).\n" +
                "- Le périphérique Bluetooth n'est pas connecté / appairage incomplet.\n\n" +
                "Solutions :\n" +
                "1) Ferme tout ce qui peut utiliser le COM (Serial Monitor, etc.).\n" +
                "2) Dans Windows > Bluetooth > Ports COM, choisis le port \"Outgoing\" du HC-06.\n" +
                "3) Clique Refresh puis réessaie.\n" +
                "4) Sinon supprime l'appareil Bluetooth et ré-appaire.",
                "Bluetooth COM - Accès refusé",
                MessageBoxButtons.OK,
                MessageBoxIcon.Warning);
            return;
        }
        catch (Exception ex)
        {
            _portSerie.Dispose();
            _portSerie = null;
            _labelStatut.Text = $"Cannot open {portName}";
            GestionErreurs.Signaler(ex, $"Erreur ouverture port {portName}", this, afficherDialogue: true);
            return;
        }

        MettreAJourEtatConnexion(true);
        _nomPortConnecte = portName;
        _derniereTelemetrieA = DateTime.MinValue;
        _labelStatut.Text = $"Connected: {portName} @{VitesseBaud} (click Stream (M))";
        Journaliser("--", $"Connected {portName} @{VitesseBaud}");
        NotifierAction($"Connecté à {portName} @{VitesseBaud}");
        MettreAJourTableauDeBord();
    }

    /// <summary>
    /// Ferme proprement le port série et remet l'UI dans l'état déconnecté.
    /// </summary>
    private void Deconnecter()
    {
        if (_portSerie is not null)
        {
            try { _portSerie.Close(); } catch { /* ignore */ }
            try { _portSerie.Dispose(); } catch { /* ignore */ }
        }

        _portSerie = null;
        _nomPortConnecte = null;
        MettreAJourEtatConnexion(false);
        _labelStatut.Text = "Disconnected";

        Journaliser("--", "Disconnected");
        MettreAJourTableauDeBord();
    }

    /// <summary>
    /// Envoie une commande ASCII au robot (ligne terminée par '\n').
    /// </summary>
    private void EnvoyerLigne(string s)
    {
        try
        {
            _portSerie?.Write(s);
            _lignesTx++;
            Journaliser("TX", s.Trim());
            MettreAJourTexteStatut();
        }
        catch (Exception ex)
        {
            GestionErreurs.Signaler(ex, "Écriture port série", this, afficherDialogue: false);
        }
    }

    /// <summary>
    /// Traite une ligne ASCII complète reçue depuis le robot.
    /// Les formats supportés sont documentés en en-tête de classe.
    /// </summary>
    private void TraiterLigneRecue(string line)
    {
        line = line.Trim();
        if (line.Length == 0) return;

        _lignesRx++;
        Journaliser("RX", line);

        // T,<ms>,<x_mm>,<y_mm>,<yaw_cdeg>,<dist_mm>,<mode>
        if (line.StartsWith("T,"))
        {
            var parts = line.Split(',');
            if (parts.Length < 7) return;

            if (!int.TryParse(parts[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out _positionXMm)) return;
            if (!int.TryParse(parts[3], NumberStyles.Integer, CultureInfo.InvariantCulture, out _positionYMm)) return;
            if (!int.TryParse(parts[4], NumberStyles.Integer, CultureInfo.InvariantCulture, out _capCdeg)) return;
            if (!int.TryParse(parts[5], NumberStyles.Integer, CultureInfo.InvariantCulture, out _distanceMm)) return;
            if (!int.TryParse(parts[6], NumberStyles.Integer, CultureInfo.InvariantCulture, out _mode)) return;

            if (_distanceMm > 0)
                MettreAJourCarteAvecMesure(_positionXMm, _positionYMm, _capCdeg, _distanceMm);

            _derniereTelemetrieA = DateTime.Now;
            Journaliser("T", $"x={_positionXMm} y={_positionYMm} yaw={_capCdeg / 100.0:F1}° dist={_distanceMm} mode={_mode}({ModeToText(_mode)})");
            MettreAJourTexteStatut();
            DemanderRafraichissementCarte();
            return;
        }

        // D,<dist_mm>
        if (line.StartsWith("D,"))
        {
            var parts = line.Split(',');
            if (parts.Length >= 2 && int.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var d))
            {
                _distanceMm = d;
                Journaliser("D", $"dist={_distanceMm}mm");
            }
            MettreAJourTexteStatut();
            DemanderRafraichissementCarte();
        }
    }

    /// <summary>
    /// Met à jour la carte à partir d'une mesure distance (télémètre) depuis la pose courante.
    /// Trace un rayon de cellules libres et marque la cellule d'impact comme occupée.
    /// </summary>
    private void MettreAJourCarteAvecMesure(int xMm, int yMm, int yawCdeg, int distMm)
    {
        // endpoint in world
        var yawRad = (yawCdeg / 100.0) * (Math.PI / 180.0);
        var ex = xMm + (int)(distMm * Math.Cos(yawRad));
        var ey = yMm + (int)(distMm * Math.Sin(yawRad));

        var (sx, sy) = MondeVersGrille(xMm, yMm);
        var (gx, gy) = MondeVersGrille(ex, ey);

        TracerRayon(sx, sy, gx, gy);
    }

    private static int Clamp(int v, int min, int max) => v < min ? min : (v > max ? max : v);

    private static Color ColorForCell(sbyte v)
    {
        return v switch
        {
            0 => Color.White,
            1 => Color.Black,
            _ => Color.FromArgb(40, 40, 40)
        };
    }

    /// <summary>
    /// Convertit des coordonnées monde (mm) en indices de grille.
    /// Convention : l'origine (0,0) est au centre de la grille.
    /// </summary>
    private (int gx, int gy) MondeVersGrille(int xMm, int yMm)
    {
        var cx = GridW / 2;
        var cy = GridH / 2;
        var gx = cx + (int)Math.Round(xMm / (double)CellSizeMm);
        var gy = cy - (int)Math.Round(yMm / (double)CellSizeMm);
        gx = Clamp(gx, 0, GridW - 1);
        gy = Clamp(gy, 0, GridH - 1);
        return (gx, gy);
    }

    /// <summary>
    /// Convertit un index de grille en coordonnées monde (mm).
    /// </summary>
    private (int xMm, int yMm) GrilleVersMonde(int gx, int gy)
    {
        var cx = GridW / 2;
        var cy = GridH / 2;
        var xMm = (gx - cx) * CellSizeMm;
        var yMm = (cy - gy) * CellSizeMm;
        return (xMm, yMm);
    }

    private void DefinirCellule(int gx, int gy, sbyte value)
    {
        if (gx < 0 || gy < 0 || gx >= GridW || gy >= GridH) return;

        if (value == 0 && _grille[gx, gy] == 1)
            return; // ne pas effacer un obstacle par "free"

        _grille[gx, gy] = value;
        _bitmapGrille.SetPixel(gx, gy, ColorForCell(value));
    }

    private void TracerRayon(int x0, int y0, int x1, int y1)
    {
        // Bresenham
        int dx = Math.Abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -Math.Abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int err = dx + dy;

        int x = x0;
        int y = y0;

        // mark free along the ray, occupied at end
        while (true)
        {
            if (x == x1 && y == y1)
                break;

            DefinirCellule(x, y, 0);

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

            if (x < 0 || y < 0 || x >= GridW || y >= GridH)
                break;
        }

        DefinirCellule(x1, y1, 1);
    }

    private void PanneauCarteOnMouseClick(object? sender, MouseEventArgs e)
    {
        GestionErreurs.Executer(() =>
        {
            // Clic dans la carte => envoi d'un objectif (G,x,y) au robot.
            var panelRect = _panneauCarte.ClientRectangle;
            if (panelRect.Width <= 0 || panelRect.Height <= 0) return;

        // Map is drawn stretched to panel
            var gx = (int)Math.Round(e.X / (double)panelRect.Width * (GridW - 1));
            var gy = (int)Math.Round(e.Y / (double)panelRect.Height * (GridH - 1));
            gx = Clamp(gx, 0, GridW - 1);
            gy = Clamp(gy, 0, GridH - 1);

            var (wx, wy) = GrilleVersMonde(gx, gy);
            _objectifXMm = wx;
            _objectifYMm = wy;
            _objectifDefini = true;

            EnvoyerLigne($"G,{wx},{wy}\n");
            Journaliser("--", $"Goal click -> G,{wx},{wy}");
            NotifierAction($"Objectif envoyé : x={wx} y={wy}");

            DemanderRafraichissementCarte();
        }, "Clic sur la carte", this, afficherDialogue: false);
    }

    private void PanneauCarteOnPaint(object? sender, PaintEventArgs e)
    {
        GestionErreurs.Executer(() =>
        {
            e.Graphics.Clear(Color.Black);
            e.Graphics.InterpolationMode = InterpolationMode.NearestNeighbor;
            e.Graphics.PixelOffsetMode = PixelOffsetMode.Half;

            var dst = _panneauCarte.ClientRectangle;
            if (dst.Width <= 0 || dst.Height <= 0) return;

            // La bitmap fait exactement GridW x GridH pixels, qu'on étire à l'écran.
            e.Graphics.DrawImage(_bitmapGrille, dst);

            // Dessin de la pose robot
            var (rx, ry) = MondeVersGrille(_positionXMm, _positionYMm);
            var px = dst.Left + (int)Math.Round(rx / (double)(GridW - 1) * dst.Width);
            var py = dst.Top + (int)Math.Round(ry / (double)(GridH - 1) * dst.Height);

            using var robotBrush = new SolidBrush(Color.Red);
            e.Graphics.FillEllipse(robotBrush, px - 4, py - 4, 8, 8);

            // Trait d'orientation
            var yawRad = (_capCdeg / 100.0) * (Math.PI / 180.0);
            var hx = px + (int)(18 * Math.Cos(yawRad));
            var hy = py - (int)(18 * Math.Sin(yawRad));
            using var pen = new Pen(Color.Red, 2);
            e.Graphics.DrawLine(pen, px, py, hx, hy);

            // Objectif (croix verte)
            if (_objectifDefini)
            {
                var (gx, gy) = MondeVersGrille(_objectifXMm, _objectifYMm);
                var gpx = dst.Left + (int)Math.Round(gx / (double)(GridW - 1) * dst.Width);
                var gpy = dst.Top + (int)Math.Round(gy / (double)(GridH - 1) * dst.Height);
                using var gpen = new Pen(Color.Lime, 2);
                e.Graphics.DrawLine(gpen, gpx - 6, gpy, gpx + 6, gpy);
                e.Graphics.DrawLine(gpen, gpx, gpy - 6, gpx, gpy + 6);
            }

            // Indicateur de mode
            using var modeBrush = new SolidBrush(Color.Yellow);
            e.Graphics.DrawString($"Mode={_mode}", Font, modeBrush, 8, 8);
        }, "Dessin de la carte", this, afficherDialogue: false);
    }

    protected override void OnFormClosed(FormClosedEventArgs e)
    {
        Deconnecter();
        base.OnFormClosed(e);
    }

    private void MainFormOnKeyDown(object? sender, KeyEventArgs e)
    {
        if (_portSerie is null || !_portSerie.IsOpen)
            return;

        if (_touchesEnfoncees.Contains(e.KeyCode))
            return;

        // Mapping clavier demandé: touches F B S L R
        // On envoie la lettre correspondante.
        switch (e.KeyCode)
        {
            case Keys.F:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=F";
                Journaliser("KEY", "Down F -> F");
                EnvoyerLigne("F\n");
                NotifierAction("Commande clavier : F (avant)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.B:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=B";
                Journaliser("KEY", "Down B -> B");
                EnvoyerLigne("B\n");
                NotifierAction("Commande clavier : B (arrière)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.L:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=L";
                Journaliser("KEY", "Down L -> L");
                EnvoyerLigne("L\n");
                NotifierAction("Commande clavier : L (gauche)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.R:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=R";
                Journaliser("KEY", "Down R -> R");
                EnvoyerLigne("R\n");
                NotifierAction("Commande clavier : R (droite)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.S:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=S";
                Journaliser("KEY", "Down S -> S");
                EnvoyerLigne("S\n");
                NotifierAction("Commande clavier : S (stop)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;

            // Flèches directionnelles : mapping vers F/B/L/R
            case Keys.Up:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=Up";
                Journaliser("KEY", "Down Up -> F");
                EnvoyerLigne("F\n");
                NotifierAction("Commande clavier : ↑ -> F (avant)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.Down:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=Down";
                Journaliser("KEY", "Down Down -> B");
                EnvoyerLigne("B\n");
                NotifierAction("Commande clavier : ↓ -> B (arrière)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.Left:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=Left";
                Journaliser("KEY", "Down Left -> L");
                EnvoyerLigne("L\n");
                NotifierAction("Commande clavier : ← -> L (gauche)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.Right:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=Right";
                Journaliser("KEY", "Down Right -> R");
                EnvoyerLigne("R\n");
                NotifierAction("Commande clavier : → -> R (droite)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;

            // Ping clavier
            case Keys.P:
                _touchesEnfoncees.Add(e.KeyCode);
                _infoDerniereTouche = "| KEY=P";
                Journaliser("KEY", "Down P -> P (ping)");
                EnvoyerLigne("P\n");
                NotifierAction("Commande clavier : P (ping)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
        }
    }

    private void MainFormOnKeyUp(object? sender, KeyEventArgs e)
    {
        if (_portSerie is null || !_portSerie.IsOpen)
            return;

        if (!_touchesEnfoncees.Remove(e.KeyCode))
            return;

        // Au relâchement d'une touche de mouvement, on stop.
        // (évite un robot qui continue si tu lâches la touche)
        switch (e.KeyCode)
        {
            case Keys.F:
            case Keys.B:
            case Keys.L:
            case Keys.R:
            case Keys.Up:
            case Keys.Down:
            case Keys.Left:
            case Keys.Right:
                _infoDerniereTouche = "| KEY=UP";
                Journaliser("KEY", $"Up {e.KeyCode} -> S");
                EnvoyerLigne("S\n");
                NotifierAction("Commande clavier : relâché -> S (stop)");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
        }
    }

    private sealed class DoubleBufferedPanel : Panel
    {
        public DoubleBufferedPanel()
        {
            DoubleBuffered = true;
            ResizeRedraw = true;
        }
    }
}
