using System;
using System.Globalization;
using System.IO;
using System.Text;
using System.Threading;
using System.Windows.Forms;
using System.Diagnostics;

namespace RobotMapper;

/// <summary>
/// Centralise la gestion des erreurs : journalisation sur disque + affichage utilisateur.
/// Objectif : éviter les try/catch dispersés et ne jamais faire planter l'UI sur une exception.
/// </summary>
internal static class GestionErreurs
{
    private static readonly object VerrouLog = new();

    /// <summary>
    /// Exécute une action et capture toute exception pour la journaliser.
    /// </summary>
    /// <param name="action">Action à exécuter.</param>
    /// <param name="contexte">Texte court décrivant le contexte fonctionnel (ex: "Connexion port série").</param>
    /// <param name="fenetreParente">Fenêtre parente pour l'éventuel MessageBox.</param>
    /// <param name="afficherDialogue">Affiche un dialogue utilisateur si vrai.</param>
    public static void Executer(Action action, string contexte, IWin32Window? fenetreParente = null, bool afficherDialogue = true)
    {
        try
        {
            action();
        }
        catch (Exception ex)
        {
            Signaler(ex, contexte, fenetreParente, afficherDialogue);
        }
    }

    /// <summary>
    /// Journalise une exception dans un fichier de log et, optionnellement, affiche un dialogue.
    /// Les logs sont stockés dans : %LocalAppData%\RobotMapper\logs.
    /// </summary>
    /// <param name="exception">Exception à journaliser.</param>
    /// <param name="contexte">Contexte fonctionnel.</param>
    /// <param name="fenetreParente">Fenêtre parente pour le MessageBox.</param>
    /// <param name="afficherDialogue">Affiche un dialogue utilisateur si vrai.</param>
    public static void Signaler(Exception exception, string contexte, IWin32Window? fenetreParente = null, bool afficherDialogue = true)
    {
        try
        {
            var cheminLog = EcrireLog(exception, contexte);

            // Sortie diagnostic (utile en dev / capture via DebugView).
            try
            {
                Debug.WriteLine($"[RobotMapper] {contexte}: {exception}");
                Trace.WriteLine($"[RobotMapper] {contexte}: {exception}");
            }
            catch
            {
                // Ignore
            }

            if (!afficherDialogue)
                return;

            var message = new StringBuilder();
            message.AppendLine("Une erreur est survenue.");
            message.AppendLine();
            message.AppendLine($"Contexte : {contexte}");
            message.AppendLine($"Type : {exception.GetType().FullName}");
            message.AppendLine($"Détails : {exception.Message}");
            if (!string.IsNullOrWhiteSpace(exception.StackTrace))
            {
                message.AppendLine();
                message.AppendLine("Pile (extrait) :");

                var lignes = exception.StackTrace
                    .Split(new[] { "\r\n", "\n" }, StringSplitOptions.RemoveEmptyEntries);

                // On affiche seulement les premières lignes, le reste est dans le fichier log.
                for (var i = 0; i < Math.Min(8, lignes.Length); i++)
                    message.AppendLine(lignes[i]);
            }
            message.AppendLine();
            message.AppendLine("Un fichier de log a été créé :");
            message.AppendLine(cheminLog);

            // Si on est sur un thread non-UI, on bascule sur le thread UI si possible.
            if (Application.MessageLoop && SynchronizationContext.Current is not null)
            {
                SynchronizationContext.Current.Post(_ =>
                {
                    MessageBox.Show(
                        fenetreParente,
                        message.ToString(),
                        "RobotMapper - Erreur",
                        MessageBoxButtons.OK,
                        MessageBoxIcon.Error);
                }, null);
            }
            else
            {
                MessageBox.Show(
                    fenetreParente,
                    message.ToString(),
                    "RobotMapper - Erreur",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error);
            }
        }
        catch
        {
            // En dernier recours : ne jamais relancer d'exception depuis la gestion d'erreurs.
        }
    }

    private static string EcrireLog(Exception exception, string contexte)
    {
        lock (VerrouLog)
        {
            var dossier = Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                "RobotMapper",
                "logs");

            Directory.CreateDirectory(dossier);

            var horodatage = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss", CultureInfo.InvariantCulture);
            var chemin = Path.Combine(dossier, $"RobotMapper_{horodatage}.log");

            var contenu = new StringBuilder();
            contenu.AppendLine($"Date : {DateTime.Now:O}");
            contenu.AppendLine($"Contexte : {contexte}");
            contenu.AppendLine($"OS : {Environment.OSVersion}");
            contenu.AppendLine($".NET : {Environment.Version}");
            contenu.AppendLine();
            contenu.AppendLine(exception.ToString());

            File.WriteAllText(chemin, contenu.ToString(), Encoding.UTF8);
            return chemin;
        }
    }
}
