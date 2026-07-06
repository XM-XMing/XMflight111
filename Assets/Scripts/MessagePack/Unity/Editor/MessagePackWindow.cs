#nullable disable
#if UNITY_EDITOR

using System;
using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;

namespace MessagePack.Unity.Editor
{
    internal class MessagePackWindow : EditorWindow
    {
        static MessagePackWindow window;

        bool processInitialized;

        bool isDotnetInstalled;
        string dotnetVersion;

        bool isInstalledMpc;
        bool installingMpc;
        bool invokingMpc;

        MpcArgument mpcArgument;

        [MenuItem("Window/MessagePack/CodeGenerator")]
        public static void OpenWindow()
        {
            if (window != null)
            {
                window.Close();
            }

            GetWindow<MessagePackWindow>("MessagePack CodeGen").Show();
        }

        async void OnEnable()
        {
            window = this;

            try
            {
                var dotnet = await ProcessHelper.FindDotnetAsync();
                isDotnetInstalled = dotnet.found;
                dotnetVersion = dotnet.version;

                if (isDotnetInstalled)
                {
                    isInstalledMpc = await ProcessHelper.IsInstalledMpc();
                }
            }
            catch (Exception ex)
            {
                UnityEngine.Debug.LogException(ex);
            }
            finally
            {
                mpcArgument = MpcArgument.Restore();
                processInitialized = true;
                Repaint();
            }
        }

        void OnGUI()
        {
            if (!processInitialized)
            {
                GUILayout.Label("Check .NET Core SDK/CodeGen install status.");
                return;
            }

            if (mpcArgument == null)
            {
                return;
            }

            if (!isDotnetInstalled)
            {
                GUILayout.Label(".NET Core SDK not found.");
                GUILayout.Label("MessagePack CodeGen requires .NET Core Runtime.");

                if (GUILayout.Button("Open .NET Core install page."))
                {
                    Application.OpenURL("https://dotnet.microsoft.com/download");
                }

                return;
            }

            GUILayout.Label(".NET Core SDK: " + dotnetVersion);

            if (!isInstalledMpc)
            {
                GUILayout.Label("MessagePack CodeGen is not installed.");

                using (new EditorGUI.DisabledScope(installingMpc))
                {
                    if (GUILayout.Button("Install MessagePack CodeGen."))
                    {
                        _ = InstallMpcAsync();
                    }
                }

                return;
            }

            EditorGUILayout.LabelField("-i input path(csproj or directory):");
            TextField(mpcArgument, x => x.Input, (x, y) => x.Input = y);

            EditorGUILayout.LabelField("-o output filepath(.cs) or directory(multiple):");
            TextField(mpcArgument, x => x.Output, (x, y) => x.Output = y);

            EditorGUILayout.LabelField("-m(optional) use map mode:");
            var newToggle = EditorGUILayout.Toggle(mpcArgument.UseMapMode);
            if (mpcArgument.UseMapMode != newToggle)
            {
                mpcArgument.UseMapMode = newToggle;
                mpcArgument.Save();
            }

            EditorGUILayout.LabelField("-c(optional) conditional compiler symbols(split with ','):");
            TextField(mpcArgument, x => x.ConditionalSymbol, (x, y) => x.ConditionalSymbol = y);

            EditorGUILayout.LabelField("-r(optional) generated resolver name:");
            TextField(mpcArgument, x => x.ResolverName, (x, y) => x.ResolverName = y);

            EditorGUILayout.LabelField("-n(optional) namespace root name:");
            TextField(mpcArgument, x => x.Namespace, (x, y) => x.Namespace = y);

            EditorGUILayout.LabelField("-ms(optional) Generate #if-- files by symbols, split with ','");
            TextField(mpcArgument, x => x.MultipleIfDirectiveOutputSymbols, (x, y) => x.MultipleIfDirectiveOutputSymbols = y);

            using (new EditorGUI.DisabledScope(invokingMpc))
            {
                if (GUILayout.Button("Generate"))
                {
                    _ = GenerateAsync();
                }
            }
        }

        async Task InstallMpcAsync()
        {
            if (installingMpc)
            {
                return;
            }

            installingMpc = true;
            Repaint();

            try
            {
                var log = await ProcessHelper.InstallMpc();

                if (!string.IsNullOrWhiteSpace(log))
                {
                    UnityEngine.Debug.Log(log);
                }

                isInstalledMpc = !(log != null && log.Contains("error"));
            }
            catch (Exception ex)
            {
                isInstalledMpc = false;
                UnityEngine.Debug.LogException(ex);
            }
            finally
            {
                installingMpc = false;
                Repaint();
            }
        }

        async Task GenerateAsync()
        {
            if (invokingMpc)
            {
                return;
            }

            invokingMpc = true;
            Repaint();

            try
            {
                var commandLineArguments = mpcArgument.ToString();
                UnityEngine.Debug.Log("Generate MessagePack Files, command:" + commandLineArguments);

                var log = await ProcessHelper.InvokeProcessStartAsync("mpc", commandLineArguments);
                UnityEngine.Debug.Log(log);
            }
            catch (Exception ex)
            {
                UnityEngine.Debug.LogException(ex);
            }
            finally
            {
                invokingMpc = false;
                Repaint();
            }
        }

        void TextField(MpcArgument args, Func<MpcArgument, string> getter, Action<MpcArgument, string> setter)
        {
            var current = getter(args);
            var newValue = EditorGUILayout.TextField(current);

            if (newValue != current)
            {
                setter(args, newValue);
                args.Save();
            }
        }
    }

    internal class MpcArgument
    {
        public string Input;
        public string Output;
        public string ConditionalSymbol;
        public string ResolverName;
        public string Namespace;
        public bool UseMapMode;
        public string MultipleIfDirectiveOutputSymbols;

        static string Key => "MessagePackCodeGen." + Application.productName;

        public static MpcArgument Restore()
        {
            if (EditorPrefs.HasKey(Key))
            {
                var json = EditorPrefs.GetString(Key);
                return JsonUtility.FromJson<MpcArgument>(json);
            }

            return new MpcArgument();
        }

        public void Save()
        {
            var json = JsonUtility.ToJson(this);
            EditorPrefs.SetString(Key, json);
        }

        public override string ToString()
        {
            var sb = new StringBuilder();

            sb.Append("-i ");
            sb.Append(Input);

            sb.Append(" -o ");
            sb.Append(Output);

            if (!string.IsNullOrWhiteSpace(ConditionalSymbol))
            {
                sb.Append(" -c ");
                sb.Append(ConditionalSymbol);
            }

            if (!string.IsNullOrWhiteSpace(ResolverName))
            {
                sb.Append(" -r ");
                sb.Append(ResolverName);
            }

            if (UseMapMode)
            {
                sb.Append(" -m");
            }

            if (!string.IsNullOrWhiteSpace(Namespace))
            {
                sb.Append(" -n ");
                sb.Append(Namespace);
            }

            if (!string.IsNullOrWhiteSpace(MultipleIfDirectiveOutputSymbols))
            {
                sb.Append(" -ms ");
                sb.Append(MultipleIfDirectiveOutputSymbols);
            }

            return sb.ToString();
        }
    }

    internal static class ProcessHelper
    {
        const string InstallName = "messagepack.generator";

        public static async Task<bool> IsInstalledMpc()
        {
            var list = await InvokeProcessStartAsync("dotnet", "tool list -g");
            return list.Contains(InstallName);
        }

        public static async Task<string> InstallMpc()
        {
            return await InvokeProcessStartAsync("dotnet", "tool install --global " + InstallName);
        }

        public static async Task<(bool found, string version)> FindDotnetAsync()
        {
            try
            {
                var version = await InvokeProcessStartAsync("dotnet", "--version");
                return (true, version);
            }
            catch
            {
                return (false, null);
            }
        }

        public static Task<string> InvokeProcessStartAsync(string fileName, string arguments)
        {
            var psi = new ProcessStartInfo()
            {
                CreateNoWindow = true,
                WindowStyle = ProcessWindowStyle.Hidden,
                StandardOutputEncoding = Encoding.UTF8,
                StandardErrorEncoding = Encoding.UTF8,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                UseShellExecute = false,
                FileName = fileName,
                Arguments = arguments,
                WorkingDirectory = Application.dataPath
            };

            Process p;

            try
            {
                p = Process.Start(psi);
            }
            catch (Exception ex)
            {
                return Task.FromException<string>(ex);
            }

            var tcs = new TaskCompletionSource<string>();

            p.EnableRaisingEvents = true;
            p.Exited += (object sender, EventArgs e) =>
            {
                var output = p.StandardOutput.ReadToEnd();
                var error = p.StandardError.ReadToEnd();

                p.Dispose();

                if (!string.IsNullOrWhiteSpace(error))
                {
                    tcs.TrySetResult(output + "\n" + error);
                }
                else
                {
                    tcs.TrySetResult(output);
                }
            };

            return tcs.Task;
        }
    }
}

#endif
