using UnityEngine;
using System.IO;
using System.Collections.Generic;

public class PLYImporter_MRI : MonoBehaviour
{
    void Start()
    {
        // Assetsフォルダの絶対パスを取得し、指定のファイルのパスを追加
        string absolutePath = Application.dataPath + "/PointCloud/real_env_corn_ground.ply";

        // 絶対パスをログに表示
        Debug.Log("絶対パス: " + absolutePath);

        // ファイルが存在するか確認
        if (File.Exists(absolutePath))
        {
            Debug.Log("ファイルが見つかりました。");

            // メッシュをインポート
            Mesh mesh = ImportPLY(absolutePath);
            if (mesh != null)
            {
                GameObject meshObject = new GameObject("Imported Mesh");
                MeshFilter meshFilter = meshObject.AddComponent<MeshFilter>();
                meshFilter.mesh = mesh;

                // MeshRendererを追加してデフォルトマテリアルを設定
                MeshRenderer meshRenderer = meshObject.AddComponent<MeshRenderer>();
                meshRenderer.material = new Material(Shader.Find("Standard"));

                // メッシュオブジェクトのスケールを調整
                meshObject.transform.localScale = new Vector3(10, 10, 10);

                // メッシュオブジェクトをカメラの正面に配置
                meshObject.transform.position = new Vector3(0, 0, 0);

                Debug.Log("メッシュのインポートが完了しました。");
            }
            else
            {
                Debug.LogError("メッシュのインポートに失敗しました。");
            }
        }
        else
        {
            Debug.LogError("ファイルが見つかりません。絶対パスを確認してください。");
        }
    }

    Mesh ImportPLY(string filePath)
    {
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();  // この部分は後ほど使用される可能性があります

        try
        {
            using (StreamReader reader = new StreamReader(filePath))
            {
                string line;

                while ((line = reader.ReadLine()) != null)
                {
                    // 'v'で始まる行は頂点情報
                    if (line.StartsWith("v "))
                    {
                        string[] parts = line.Split(' ');
                        float x = float.Parse(parts[1]);
                        float y = float.Parse(parts[2]);
                        float z = float.Parse(parts[3]);
                        vertices.Add(new Vector3(x, y, z));
                    }
                }
            }

            Debug.Log("Vertices Count: " + vertices.Count);

            if (vertices.Count > 0)
            {
                Mesh mesh = new Mesh();
                mesh.vertices = vertices.ToArray();
                mesh.triangles = GenerateTriangles(vertices.Count);
                mesh.RecalculateNormals();
                return mesh;
            }
        }
        catch (System.Exception ex)
        {
            Debug.LogError("ファイルの読み込み中にエラーが発生しました: " + ex.Message);
        }

        return null;
    }

    // シンプルな方法で三角形を生成（これはデモ用）
    int[] GenerateTriangles(int vertexCount)
    {
        List<int> triangles = new List<int>();

        for (int i = 0; i < vertexCount - 2; i += 3)
        {
            triangles.Add(i);
            triangles.Add(i + 1);
            triangles.Add(i + 2);
        }

        return triangles.ToArray();
    }
}
