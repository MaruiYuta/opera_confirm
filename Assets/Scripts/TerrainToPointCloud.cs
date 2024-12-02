using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;

public class TerrainToPointCloud : MonoBehaviour
{
    public Terrain terrain;
    public float pointSpacing = 0.1f; // 点の間隔
    public string outputFilePath = "Assets/TerrainPointcloud/"; // 保存先のファイルパス
    public GameObject pointPrefab; // 点を表示するためのプレハブ

    public Vector2 rangeMin = new Vector2(0, 0); // 範囲の最小座標
    public Vector2 rangeMax = new Vector2(10, 10); // 範囲の最大座標

    private List<Vector3> pointCloud; // クラスフィールドとして宣言
    private List<GameObject> pointObjects = new List<GameObject>(); // 表示された点のリスト

    void Start()
    {
        Debug.Log("Start method called."); 
        pointCloud = GeneratePointCloud(terrain, pointSpacing);
        SavePointCloudToFile(pointCloud, outputFilePath);
        VisualizePointCloud(pointCloud);
        Debug.Log($"Point cloud saved to {outputFilePath}");
    }

    List<Vector3> GeneratePointCloud(Terrain terrain, float spacing)
    {
        List<Vector3> points = new List<Vector3>();
        TerrainData terrainData = terrain.terrainData;
        Vector3 terrainPosition = terrain.transform.position;

        for (float x = rangeMin.x; x < rangeMax.x; x += spacing)
        {
            for (float z = rangeMin.y; z < rangeMax.y; z += spacing)
            {
                Vector3 worldPos = new Vector3(x + terrainPosition.x, 0, z + terrainPosition.z);
                float y = terrain.SampleHeight(worldPos) + terrainPosition.y;
                points.Add(new Vector3(worldPos.x, y, worldPos.z));
            }
        }

        return points;
    }

    void SavePointCloudToFile(List<Vector3> pointCloud, string filePath)
    {
        StringBuilder sb = new StringBuilder();

        foreach (Vector3 point in pointCloud)
        {
            sb.AppendLine($"{point.x} {point.y} {point.z}");
        }

        File.WriteAllText(filePath, sb.ToString());
        Debug.Log($"Point cloud saved to {filePath}");
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.G)) // Gキーを押すと実行される,点群保存
        {
            Debug.Log("G key pressed");
            pointCloud = GeneratePointCloud(terrain, pointSpacing);
            SavePointCloudToFile(pointCloud, outputFilePath);
        }
        if (Input.GetKeyDown(KeyCode.F)) // Fキーを押すと実行される,点群表示
        {
            Debug.Log("F key pressed");
            RefreshPointCloudVisualization();
        }
    }

    void RefreshPointCloudVisualization()
    {
        Debug.Log("Refreshing point cloud visualization");

        // 既存の点群オブジェクトを削除
        foreach (GameObject pointObj in pointObjects)
        {
            Destroy(pointObj);
        }
        pointObjects.Clear();

        // 新しい点群を生成し表示
        pointCloud = GeneratePointCloud(terrain, pointSpacing);
        VisualizePointCloud(pointCloud);
        Debug.Log("Point cloud visualization refreshed");
    }

    void VisualizePointCloud(List<Vector3> pointCloud)
    {
        foreach (Vector3 point in pointCloud)
        {
            GameObject pointObj = Instantiate(pointPrefab, point, Quaternion.identity);
            pointObjects.Add(pointObj);
        }
    }
}
