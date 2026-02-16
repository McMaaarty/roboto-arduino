using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Windows.Forms;

namespace RobotMapper;

public sealed class MainForm : Form
{
    private readonly ComboBox _ports = new() { DropDownStyle = ComboBoxStyle.DropDownList, Width = 140 };
    private readonly Button _refresh = new() { Text = "Refresh", Width = 80 };
    private readonly Button _connect = new() { Text = "Connect", Width = 80 };
    private readonly Button _disconnect = new() { Text = "Disconnect", Width = 80, Enabled = false };
    private readonly Button _toggleStream = new() { Text = "Stream (M)", Width = 100, Enabled = false };
    private readonly Button _toggleAuto = new() { Text = "Auto (A)", Width = 90, Enabled = false };
    private readonly Button _ping = new() { Text = "Ping (P)", Width = 90, Enabled = false };
    private readonly Label _status = new() { AutoSize = true, Text = "Disconnected" };
    private readonly DoubleBufferedPanel _mapPanel = new() { Dock = DockStyle.Fill, BackColor = Color.Black };
    private readonly TextBox _log = new()
    {
        Dock = DockStyle.Bottom,
        Height = 180,
        Multiline = true,
        ReadOnly = true,
        ScrollBars = ScrollBars.Vertical,
        Font = new Font("Consolas", 9f),
    };

    private SerialPort? _sp;
    private string? _connectedPort;

    private readonly object _rxLock = new();
    private readonly StringBuilder _rxBuf = new();

    // Grid map
    private const int GridW = 201;
    private const int GridH = 201;
    private const int CellSizeMm = 50; // 5cm
    private readonly sbyte[,] _grid = new sbyte[GridW, GridH]; // -1 unknown, 0 free, 1 occupied
    private readonly Bitmap _gridBitmap = new(GridW, GridH);

    // Telemetry state
    private int _xMm;
    private int _yMm;
    private int _yawCdeg;
    private int _distMm;
    private int _mode;
    private bool _hasGoal;
    private int _goalXmm;
    private int _goalYmm;

    private int _rxLines;
    private int _txLines;
    private bool _pendingInvalidate;

    private DateTime _lastTelemetryAt = DateTime.MinValue;
    private string _lastKeyInfo = "";

    private readonly HashSet<Keys> _keysDown = new();

    private readonly System.Windows.Forms.Timer _invalidateTimer = new() { Interval = 50 };

    public MainForm()
    {
        Text = "Robot Mapper";
        Width = 1100;
        Height = 800;

        KeyPreview = true;
        DoubleBuffered = true;

        for (int x = 0; x < GridW; x++)
        for (int y = 0; y < GridH; y++)
            _grid[x, y] = -1;

        InitializeBitmap();

        var top = new FlowLayoutPanel { Dock = DockStyle.Top, Height = 40, Padding = new Padding(8), WrapContents = false };
        top.Controls.Add(new Label { Text = "COM:", AutoSize = true, TextAlign = ContentAlignment.MiddleLeft, Padding = new Padding(0, 8, 0, 0) });
        top.Controls.Add(_ports);
        top.Controls.Add(_refresh);
        top.Controls.Add(_connect);
        top.Controls.Add(_disconnect);
        top.Controls.Add(_toggleStream);
        top.Controls.Add(_toggleAuto);
        top.Controls.Add(_ping);
        top.Controls.Add(_status);

        Controls.Add(_mapPanel);
        Controls.Add(top);
        Controls.Add(_log);

        _mapPanel.Paint += MapPanelOnPaint;
        _mapPanel.MouseClick += MapPanelOnMouseClick;

        KeyDown += MainFormOnKeyDown;
        KeyUp += MainFormOnKeyUp;

        _refresh.Click += (_, _) => RefreshPorts();
        _connect.Click += (_, _) => Connect();
        _disconnect.Click += (_, _) => Disconnect();
        _toggleStream.Click += (_, _) => SendLine("M\n");
        _toggleAuto.Click += (_, _) => SendLine("A\n");
        _ping.Click += (_, _) => SendLine("P\n");

        _invalidateTimer.Tick += (_, _) =>
        {
            UpdateStatusText();
            if (_pendingInvalidate)
            {
                _pendingInvalidate = false;
                _mapPanel.Invalidate();
            }
        };
        _invalidateTimer.Start();

        RefreshPorts();
    }

    private void Log(string direction, string message)
    {
        var line = $"{DateTime.Now:HH:mm:ss.fff} {direction} {message}";
        _log.AppendText(line + Environment.NewLine);

        // Limit size (keep last ~600 lines)
        var text = _log.Text;
        var lines = text.Split(new[] { "\r\n", "\n" }, StringSplitOptions.None);
        if (lines.Length > 650)
        {
            var keep = string.Join(Environment.NewLine, lines.Skip(lines.Length - 600));
            _log.Text = keep;
            _log.SelectionStart = _log.TextLength;
            _log.ScrollToCaret();
        }
    }

    private void QueueInvalidate()
    {
        _pendingInvalidate = true;
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

    private void UpdateStatusText()
    {
        if (_sp is null || !_sp.IsOpen)
            return;

        var modeText = ModeToText(_mode);

        string ageText;
        if (_lastTelemetryAt == DateTime.MinValue)
        {
            ageText = "No telemetry (click Stream (M))";
        }
        else
        {
            var age = DateTime.Now - _lastTelemetryAt;
            ageText = $"Last T={age.TotalMilliseconds:0}ms ago";
        }

        _status.Text = $"{_connectedPort} @9600 | x={_xMm} y={_yMm} yaw={_yawCdeg / 100.0:F1}° dist={_distMm} mode={_mode}({modeText}) | {ageText} | RX={_rxLines} TX={_txLines} {_lastKeyInfo}";
    }

    private void InitializeBitmap()
    {
        for (int x = 0; x < GridW; x++)
        for (int y = 0; y < GridH; y++)
            _gridBitmap.SetPixel(x, y, Color.FromArgb(40, 40, 40));
    }

    private void RefreshPorts()
    {
        var ports = SerialPort.GetPortNames().OrderBy(p => p).ToArray();
        _ports.Items.Clear();
        _ports.Items.AddRange(ports);
        if (ports.Length > 0)
            _ports.SelectedIndex = 0;
    }

    private void Connect()
    {
        if (_ports.SelectedItem is not string portName)
            return;

        Disconnect();

        _sp = new SerialPort(portName, 9600)
        {
            NewLine = "\n",
            Encoding = Encoding.ASCII,
            ReadTimeout = 500,
            WriteTimeout = 500
        };

        _sp.DataReceived += (_, _) =>
        {
            try
            {
                if (_sp is null) return;

                var chunk = _sp.ReadExisting();
                if (string.IsNullOrEmpty(chunk))
                    return;

                List<string> lines = new();

                lock (_rxLock)
                {
                    _rxBuf.Append(chunk);

                    while (true)
                    {
                        var s = _rxBuf.ToString();
                        var idx = s.IndexOf('\n');
                        if (idx < 0)
                            break;

                        var line = s.Substring(0, idx);
                        // drop consumed + '\n'
                        _rxBuf.Clear();
                        _rxBuf.Append(s.Substring(idx + 1));

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
                            OnLine(line);
                    }));
                }
            }
            catch
            {
                // ignore read errors
            }
        };

        try
        {
            _sp.Open();
        }
        catch (UnauthorizedAccessException)
        {
            _sp.Dispose();
            _sp = null;
            _status.Text = $"Cannot open {portName}";
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
            _sp.Dispose();
            _sp = null;
            _status.Text = $"Cannot open {portName}";
            MessageBox.Show(this, ex.Message, "Erreur ouverture port", MessageBoxButtons.OK, MessageBoxIcon.Error);
            return;
        }

        _connect.Enabled = false;
        _disconnect.Enabled = true;
        _toggleStream.Enabled = true;
        _toggleAuto.Enabled = true;
        _ping.Enabled = true;
        _connectedPort = portName;
        _lastTelemetryAt = DateTime.MinValue;
        _status.Text = $"Connected: {portName} @9600 (click Stream (M))";
        Log("--", $"Connected {portName} @9600");
    }

    private void Disconnect()
    {
        if (_sp is not null)
        {
            try { _sp.Close(); } catch { /* ignore */ }
            try { _sp.Dispose(); } catch { /* ignore */ }
        }

        _sp = null;
        _connectedPort = null;
        _connect.Enabled = true;
        _disconnect.Enabled = false;
        _toggleStream.Enabled = false;
        _toggleAuto.Enabled = false;
        _ping.Enabled = false;
        _status.Text = "Disconnected";

        Log("--", "Disconnected");
    }

    private void SendLine(string s)
    {
        try
        {
            _sp?.Write(s);
            _txLines++;
            Log("TX", s.Trim());
            UpdateStatusText();
        }
        catch
        {
            // ignore
        }
    }

    private void OnLine(string line)
    {
        line = line.Trim();
        if (line.Length == 0) return;

        _rxLines++;
        Log("RX", line);

        // T,<ms>,<x_mm>,<y_mm>,<yaw_cdeg>,<dist_mm>,<mode>
        if (line.StartsWith("T,"))
        {
            var parts = line.Split(',');
            if (parts.Length < 7) return;

            if (!int.TryParse(parts[2], NumberStyles.Integer, CultureInfo.InvariantCulture, out _xMm)) return;
            if (!int.TryParse(parts[3], NumberStyles.Integer, CultureInfo.InvariantCulture, out _yMm)) return;
            if (!int.TryParse(parts[4], NumberStyles.Integer, CultureInfo.InvariantCulture, out _yawCdeg)) return;
            if (!int.TryParse(parts[5], NumberStyles.Integer, CultureInfo.InvariantCulture, out _distMm)) return;
            if (!int.TryParse(parts[6], NumberStyles.Integer, CultureInfo.InvariantCulture, out _mode)) return;

            if (_distMm > 0)
                UpdateMapWithReading(_xMm, _yMm, _yawCdeg, _distMm);

            _lastTelemetryAt = DateTime.Now;
            Log("T", $"x={_xMm} y={_yMm} yaw={_yawCdeg / 100.0:F1}° dist={_distMm} mode={_mode}({ModeToText(_mode)})");
            UpdateStatusText();
            QueueInvalidate();
            return;
        }

        // D,<dist_mm>
        if (line.StartsWith("D,"))
        {
            var parts = line.Split(',');
            if (parts.Length >= 2 && int.TryParse(parts[1], NumberStyles.Integer, CultureInfo.InvariantCulture, out var d))
            {
                _distMm = d;
                Log("D", $"dist={_distMm}mm");
            }
            UpdateStatusText();
            QueueInvalidate();
        }
    }

    private void UpdateMapWithReading(int xMm, int yMm, int yawCdeg, int distMm)
    {
        // endpoint in world
        var yawRad = (yawCdeg / 100.0) * (Math.PI / 180.0);
        var ex = xMm + (int)(distMm * Math.Cos(yawRad));
        var ey = yMm + (int)(distMm * Math.Sin(yawRad));

        var (sx, sy) = WorldToGrid(xMm, yMm);
        var (gx, gy) = WorldToGrid(ex, ey);

        MarkRay(sx, sy, gx, gy);
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

    private (int gx, int gy) WorldToGrid(int xMm, int yMm)
    {
        var cx = GridW / 2;
        var cy = GridH / 2;
        var gx = cx + (int)Math.Round(xMm / (double)CellSizeMm);
        var gy = cy - (int)Math.Round(yMm / (double)CellSizeMm);
        gx = Clamp(gx, 0, GridW - 1);
        gy = Clamp(gy, 0, GridH - 1);
        return (gx, gy);
    }

    private (int xMm, int yMm) GridToWorld(int gx, int gy)
    {
        var cx = GridW / 2;
        var cy = GridH / 2;
        var xMm = (gx - cx) * CellSizeMm;
        var yMm = (cy - gy) * CellSizeMm;
        return (xMm, yMm);
    }

    private void SetCell(int gx, int gy, sbyte value)
    {
        if (gx < 0 || gy < 0 || gx >= GridW || gy >= GridH) return;

        if (value == 0 && _grid[gx, gy] == 1)
            return; // ne pas effacer un obstacle par "free"

        _grid[gx, gy] = value;
        _gridBitmap.SetPixel(gx, gy, ColorForCell(value));
    }

    private void MarkRay(int x0, int y0, int x1, int y1)
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

            SetCell(x, y, 0);

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

        SetCell(x1, y1, 1);
    }

    private void MapPanelOnMouseClick(object? sender, MouseEventArgs e)
    {
        // click -> goal
        var panelRect = _mapPanel.ClientRectangle;
        if (panelRect.Width <= 0 || panelRect.Height <= 0) return;

        // Map is drawn stretched to panel
        var gx = (int)Math.Round(e.X / (double)panelRect.Width * (GridW - 1));
        var gy = (int)Math.Round(e.Y / (double)panelRect.Height * (GridH - 1));
        gx = Clamp(gx, 0, GridW - 1);
        gy = Clamp(gy, 0, GridH - 1);

        var (wx, wy) = GridToWorld(gx, gy);
        _goalXmm = wx;
        _goalYmm = wy;
        _hasGoal = true;

        SendLine($"G,{wx},{wy}\n");
        Log("--", $"Goal click -> G,{wx},{wy}");

        QueueInvalidate();
    }

    private void MapPanelOnPaint(object? sender, PaintEventArgs e)
    {
        e.Graphics.Clear(Color.Black);
        e.Graphics.InterpolationMode = InterpolationMode.NearestNeighbor;
        e.Graphics.PixelOffsetMode = PixelOffsetMode.Half;

        var dst = _mapPanel.ClientRectangle;
        if (dst.Width <= 0 || dst.Height <= 0) return;

        e.Graphics.DrawImage(_gridBitmap, dst);

        // draw robot pose
        var (rx, ry) = WorldToGrid(_xMm, _yMm);
        var px = dst.Left + (int)Math.Round(rx / (double)(GridW - 1) * dst.Width);
        var py = dst.Top + (int)Math.Round(ry / (double)(GridH - 1) * dst.Height);

        using var robotBrush = new SolidBrush(Color.Red);
        e.Graphics.FillEllipse(robotBrush, px - 4, py - 4, 8, 8);

        // heading line
        var yawRad = (_yawCdeg / 100.0) * (Math.PI / 180.0);
        var hx = px + (int)(18 * Math.Cos(yawRad));
        var hy = py - (int)(18 * Math.Sin(yawRad));
        using var pen = new Pen(Color.Red, 2);
        e.Graphics.DrawLine(pen, px, py, hx, hy);

        // goal
        if (_hasGoal)
        {
            var (gx, gy) = WorldToGrid(_goalXmm, _goalYmm);
            var gpx = dst.Left + (int)Math.Round(gx / (double)(GridW - 1) * dst.Width);
            var gpy = dst.Top + (int)Math.Round(gy / (double)(GridH - 1) * dst.Height);
            using var gpen = new Pen(Color.Lime, 2);
            e.Graphics.DrawLine(gpen, gpx - 6, gpy, gpx + 6, gpy);
            e.Graphics.DrawLine(gpen, gpx, gpy - 6, gpx, gpy + 6);
        }

        // mode indicator
        using var modeBrush = new SolidBrush(Color.Yellow);
        e.Graphics.DrawString($"Mode={_mode}", Font, modeBrush, 8, 8);
    }

    protected override void OnFormClosed(FormClosedEventArgs e)
    {
        Disconnect();
        base.OnFormClosed(e);
    }

    private void MainFormOnKeyDown(object? sender, KeyEventArgs e)
    {
        if (_sp is null || !_sp.IsOpen)
            return;

        if (_keysDown.Contains(e.KeyCode))
            return;

        // Mapping clavier demandé: touches F B S L R
        // On envoie la lettre correspondante.
        switch (e.KeyCode)
        {
            case Keys.F:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=F";
                Log("KEY", "Down F -> F");
                SendLine("F\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.B:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=B";
                Log("KEY", "Down B -> B");
                SendLine("B\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.L:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=L";
                Log("KEY", "Down L -> L");
                SendLine("L\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.R:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=R";
                Log("KEY", "Down R -> R");
                SendLine("R\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.S:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=S";
                Log("KEY", "Down S -> S");
                SendLine("S\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;

            // Flèches directionnelles : mapping vers F/B/L/R
            case Keys.Up:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=Up";
                Log("KEY", "Down Up -> F");
                SendLine("F\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.Down:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=Down";
                Log("KEY", "Down Down -> B");
                SendLine("B\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.Left:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=Left";
                Log("KEY", "Down Left -> L");
                SendLine("L\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
            case Keys.Right:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=Right";
                Log("KEY", "Down Right -> R");
                SendLine("R\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;

            // Ping clavier
            case Keys.P:
                _keysDown.Add(e.KeyCode);
                _lastKeyInfo = "| KEY=P";
                Log("KEY", "Down P -> P (ping)");
                SendLine("P\n");
                e.Handled = true;
                e.SuppressKeyPress = true;
                break;
        }
    }

    private void MainFormOnKeyUp(object? sender, KeyEventArgs e)
    {
        if (_sp is null || !_sp.IsOpen)
            return;

        if (!_keysDown.Remove(e.KeyCode))
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
                _lastKeyInfo = "| KEY=UP";
                Log("KEY", $"Up {e.KeyCode} -> S");
                SendLine("S\n");
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
