using UnityEditor;
using UnityEngine;

[InitializeOnLoad]
public class AutoPlayEditor : EditorWindow
{
    private static bool isPlaying = false;
    private static float playDuration = 65f; // 10 seconds
    private static float stopDuration = 5f;  // 10 seconds
    private static double startTime = 0;
    private static bool autoPlayActive = false;

    static AutoPlayEditor()
    {
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    [MenuItem("Window/AutoPlay")]
    public static void ShowWindow()
    {
        GetWindow<AutoPlayEditor>("AutoPlay");
    }

    private void OnGUI()
    {
        playDuration = EditorGUILayout.FloatField("Play Duration (seconds):", playDuration);
        stopDuration = EditorGUILayout.FloatField("Stop Duration (seconds):", stopDuration);

        if (GUILayout.Button("Start Auto Play"))
        {
            StartAutoPlay();
        }

        if (GUILayout.Button("Stop Auto Play"))
        {
            StopAutoPlay();
        }
    }

    private static void StartAutoPlay()
    {
        if (!autoPlayActive)
        {
            autoPlayActive = true;
            EditorApplication.update += Update;
            startTime = EditorApplication.timeSinceStartup;
            isPlaying = true;
            EditorApplication.isPlaying = true;
            Debug.Log("Auto Play Started");
        }
    }

    private static void StopAutoPlay()
    {
        if (autoPlayActive)
        {
            autoPlayActive = false;
            EditorApplication.update -= Update;
            EditorApplication.isPlaying = false;
            Debug.Log("Auto Play Stopped");
        }
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.EnteredPlayMode)
        {
            StartAutoPlay();
        }
    }

    private static void Update()
    {
        if (!autoPlayActive) return;

        double currentTime = EditorApplication.timeSinceStartup;
        if (isPlaying)
        {
            if (currentTime - startTime > playDuration)
            {
                EditorApplication.isPlaying = false;
                startTime = currentTime;
                isPlaying = false;
                Debug.Log("Stopped playing at: " + currentTime);
            }
        }
        else
        {
            if (currentTime - startTime > stopDuration)
            {
                EditorApplication.isPlaying = true;
                startTime = currentTime;
                isPlaying = true;
                Debug.Log("Started playing at: " + currentTime);
            }
        }
    }
}
