using System;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RobotMapper;

/// <summary>
/// Point d'entrée de l'application WinForms.
/// Installe des handlers d'exceptions globales pour éviter les crashs silencieux.
/// </summary>
internal static class Program
{
    [STAThread]
    static void Main()
    {
        // WinForms : capturer les exceptions UI et les faire remonter proprement.
        Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);

        Application.ThreadException += (_, e) =>
            GestionErreurs.Signaler(e.Exception, "Exception UI (ThreadException)", afficherDialogue: true);

        AppDomain.CurrentDomain.UnhandledException += (_, e) =>
        {
            if (e.ExceptionObject is Exception ex)
                GestionErreurs.Signaler(ex, "Exception non gérée (AppDomain)", afficherDialogue: true);
            else
                GestionErreurs.Signaler(new Exception("Exception non gérée (objet non-Exception)."), "Exception non gérée (AppDomain)", afficherDialogue: true);
        };

        TaskScheduler.UnobservedTaskException += (_, e) =>
        {
            // Exception sur Task non awaitée : on loggue, sans spammer l'utilisateur.
            GestionErreurs.Signaler(e.Exception, "Exception non observée (TaskScheduler)", afficherDialogue: false);
            e.SetObserved();
        };

        ApplicationConfiguration.Initialize();
        GestionErreurs.Executer(() => Application.Run(new MainForm()), "Démarrage application", afficherDialogue: true);
    }
}
