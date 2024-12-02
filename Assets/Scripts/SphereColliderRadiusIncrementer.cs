// using UnityEngine;
// using UnityEditor;

// [InitializeOnLoad]
// public class SphereRadiusIncrementer : MonoBehaviour
// {
//     private static bool initialized = false;
//     private SphereCollider sphereCollider;
//     private const float incrementAmount = 0.00005f;
//     public float initialRadius = 0.0f; // 初期値を設定するためのパブリック変数

//     static SphereRadiusIncrementer()
//     {
//         EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
//     }

//     void Awake()
//     {
//         // スフィアコライダーを取得
//         sphereCollider = GetComponent<SphereCollider>();
        
//         // PlayerPrefsから保存されたRadiusを読み込み、セットする
//         // 保存された値がない場合は初期値を使用する
//         float savedRadius = PlayerPrefs.GetFloat("SphereRadius", initialRadius);
//         Debug.Log($"Awake - Loaded Radius: {savedRadius}");
//         sphereCollider.radius = savedRadius;
//     }

//     private static void OnPlayModeStateChanged(PlayModeStateChange state)
//     {
//         if (state == PlayModeStateChange.EnteredPlayMode && !initialized)
//         {
//             // 初期化フラグを設定
//             initialized = true;

//             // SphereRadiusIncrementerの全インスタンスを探して増加処理を実行
//             foreach (var instance in FindObjectsOfType<SphereRadiusIncrementer>())
//             {
//                 instance.IncrementRadius();
//             }
//         }

//         if (state == PlayModeStateChange.ExitingPlayMode)
//         {
//             // プレイモード終了時に初期化フラグをリセット
//             initialized = false;
//         }
//     }

//     private void IncrementRadius()
//     {
//         // 増加量を追加
//         sphereCollider.radius += incrementAmount;
//         Debug.Log($"IncrementRadius - New Radius: {sphereCollider.radius}");

//         // 更新されたRadiusを保存
//         PlayerPrefs.SetFloat("SphereRadius", sphereCollider.radius);
//         PlayerPrefs.Save();
//         Debug.Log($"IncrementRadius - Saved Radius: {sphereCollider.radius}");
//     }

//     [ContextMenu("Reset Radius")]
//     private void ResetRadius()
//     {
//         // 初期値にリセット
//         sphereCollider.radius = initialRadius;
//         PlayerPrefs.SetFloat("SphereRadius", initialRadius);
//         PlayerPrefs.Save();
//         Debug.Log($"ResetRadius - Radius reset to: {initialRadius}");
//     }
// }
