using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;

namespace RobotMapper.Services;

/// <summary>
/// Accès au port série : connexion, réception (lignes ASCII) et envoi.
/// Cette classe ne dépend pas de l'UI.
/// </summary>
internal sealed class PortSerieService : IDisposable
{
    private readonly object _verrouReception = new();
    private readonly StringBuilder _tamponReception = new();

    private SerialPort? _portSerie;

    public event EventHandler<string>? LigneRecue;
    public event EventHandler<bool>? ConnexionChangee;
    public event EventHandler<Exception>? ErreurSurvenue;

    public string? NomPortConnecte { get; private set; }
    public int BaudRate { get; }

    public int LignesRx { get; private set; }
    public int LignesTx { get; private set; }

    public bool EstConnecte => _portSerie is not null && _portSerie.IsOpen;

    public PortSerieService(int baudRate)
    {
        BaudRate = baudRate;
    }

    public static string[] ListerPorts() => SerialPort.GetPortNames().OrderBy(p => p).ToArray();

    public void Connecter(string portName)
    {
        Deconnecter();

        _portSerie = new SerialPort(portName, BaudRate)
        {
            NewLine = "\n",
            Encoding = Encoding.ASCII,
            ReadTimeout = 500,
            WriteTimeout = 500
        };

        _portSerie.DataReceived += (_, _) =>
        {
            try
            {
                if (_portSerie is null)
                    return;

                var chunk = _portSerie.ReadExisting();
                if (string.IsNullOrEmpty(chunk))
                    return;

                List<string> lignes = new();

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
                        _tamponReception.Clear();
                        _tamponReception.Append(s.Substring(idx + 1));

                        line = line.Trim('\r');
                        if (line.Length > 0)
                            lignes.Add(line);
                    }
                }

                foreach (var ligne in lignes)
                {
                    LignesRx++;
                    LigneRecue?.Invoke(this, ligne);
                }
            }
            catch (Exception ex)
            {
                ErreurSurvenue?.Invoke(this, ex);
            }
        };

        try
        {
            _portSerie.Open();
        }
        catch (Exception ex)
        {
            try { _portSerie.Dispose(); } catch { /* ignore */ }
            _portSerie = null;
            ErreurSurvenue?.Invoke(this, ex);
            throw;
        }

        NomPortConnecte = portName;
        ConnexionChangee?.Invoke(this, true);
    }

    public void Deconnecter()
    {
        if (_portSerie is not null)
        {
            try { _portSerie.Close(); } catch { /* ignore */ }
            try { _portSerie.Dispose(); } catch { /* ignore */ }
        }

        _portSerie = null;
        NomPortConnecte = null;
        ConnexionChangee?.Invoke(this, false);
    }

    public void Envoyer(string ligne)
    {
        if (_portSerie is null || !_portSerie.IsOpen)
            return;

        try
        {
            _portSerie.Write(ligne);
            LignesTx++;
        }
        catch (Exception ex)
        {
            ErreurSurvenue?.Invoke(this, ex);
        }
    }

    public void Dispose()
    {
        Deconnecter();
    }
}
