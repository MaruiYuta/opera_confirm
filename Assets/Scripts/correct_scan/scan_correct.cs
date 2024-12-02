// using System;
// using System.Collections.Generic;
// using System.IO;
// using UnityEngine;
// using UnityEngine.SceneManagement;
// using Unity.Collections;
// using Unity.Jobs;
// using Unity.Mathematics;

// using UnitySensors.Utils.Noise;
// using UnitySensors.Data.PointCloud;

// using Random = Unity.Mathematics.Random;

// namespace UnitySensors.Sensor.LiDAR
// {
//     public class RaycastLiDARSensor : LiDARSensor<PointXYZI>
//     {
//         private Transform _transform;
//         private JobHandle _jobHandle;
//         private IUpdateRaycastCommandsJob _updateRaycastCommandsJob;
//         private IUpdateGaussianNoisesJob _updateGaussianNoisesJob;
//         private IRaycastHitsToPointsJob _raycastHitsToPointsJob;

//         private NativeArray<float3> _directions;
//         private NativeArray<RaycastCommand> _raycastCommands;
//         private NativeArray<RaycastHit> _raycastHits;
//         private NativeArray<float> _noises;

//         private List<PointXYZI> _accumulatedPoints = new List<PointXYZI>();
//         public float saveInterval = 5.0f;  // 点群を保存する間隔（秒）

//         private float _timeSinceLastSave = 0.0f;

//         // 新しいフィールドを追加
//         public float3 scanRange = new float3(2, 5, 5);  // スキャン範囲

//         protected override void Init()
//         {
//             base.Init();
//             _transform = this.transform;
//             LoadScanData();
//             SetupJobs();
//         }

//         private void LoadScanData()
//         {
//             _directions = new NativeArray<float3>(scanPattern.size, Allocator.Persistent);
//             for (int i = 0; i < scanPattern.size; i++)
//             {
//                 _directions[i] = scanPattern.scans[i];//ここで検出した点の方向ベクトルを代入
//             }
//         }

//         private void SetupJobs()
//         {
//             _raycastCommands = new NativeArray<RaycastCommand>(pointsNum, Allocator.Persistent);
//             _raycastHits = new NativeArray<RaycastHit>(pointsNum, Allocator.Persistent);
//             _noises = new NativeArray<float>(pointsNum, Allocator.Persistent);

//             _updateRaycastCommandsJob = new IUpdateRaycastCommandsJob()//コンストラクター
//             {
//                 origin = _transform.position,//rayの発射位置
//                 localToWorldMatrix = _transform.localToWorldMatrix,//ローカル座標系から世界座標系に変換(点群)
//                 maxRange = maxRange,
//                 directions = _directions,
//                 indexOffset = 0,
//                 raycastCommands = _raycastCommands
//             };

//             _updateGaussianNoisesJob = new IUpdateGaussianNoisesJob()
//             {
//                 sigma = gaussianNoiseSigma,
//                 random = new Random((uint)Environment.TickCount),
//                 noises = _noises
//             };

//             _raycastHitsToPointsJob = new IRaycastHitsToPointsJob()
//             {
//                 minRange = minRange,
//                 minRange_sqr = minRange * minRange,
//                 maxRange = maxRange,
//                 maxIntensity = maxIntensity,
//                 sensorPosition = (float3)_transform.position,
//                 scanRange = scanRange,
//                 directions = _directions,
//                 indexOffset = 0,
//                 raycastHits = _raycastHits,
//                 noises = _noises,
//                 points = pointCloud.points,
//             };
//         }

//         protected override void UpdateSensor()
//         {
//             _updateRaycastCommandsJob.origin = _transform.position;
//             _updateRaycastCommandsJob.localToWorldMatrix = _transform.localToWorldMatrix;

//             JobHandle updateRaycastCommandsJobHandle = _updateRaycastCommandsJob.Schedule(pointsNum, 1);
//             JobHandle updateGaussianNoisesJobHandle = _updateGaussianNoisesJob.Schedule(pointsNum, 1, updateRaycastCommandsJobHandle);
//             JobHandle raycastJobHandle = RaycastCommand.ScheduleBatch(_raycastCommands, _raycastHits, pointsNum, updateGaussianNoisesJobHandle);
//             _jobHandle = _raycastHitsToPointsJob.Schedule(pointsNum, 1, raycastJobHandle);

//             JobHandle.ScheduleBatchedJobs();
//             _jobHandle.Complete();

//             _updateRaycastCommandsJob.indexOffset = (_updateRaycastCommandsJob.indexOffset + pointsNum) % scanPattern.size;
//             _raycastHitsToPointsJob.indexOffset = (_raycastHitsToPointsJob.indexOffset + pointsNum) % scanPattern.size;

//             // 蓄積点群の更新
//             _accumulatedPoints.AddRange(pointCloud.points.ToArray());

//             if (onSensorUpdated != null)
//                 onSensorUpdated.Invoke();

//             // 一定期間が経過したか確認し、点群を保存
//             _timeSinceLastSave += Time.deltaTime;
//             if (_timeSinceLastSave >= saveInterval)
//             {
//                 SaveAccumulatedPointsToPLY();
//                 _accumulatedPoints.Clear();  // 点群を保存した後、リストをクリア
//                 _timeSinceLastSave = 0.0f;  // タイマーをリセット
//             }
//         }

//         private void SaveAccumulatedPointsToPLY()
//         {
//                 string directoryPath = "Assets/PointCloud";
//                 if (!Directory.Exists(directoryPath))
//                 {
//                     Directory.CreateDirectory(directoryPath);
//                 }

//             string filePath = Path.Combine(directoryPath, $"{DateTime.Now:yyyyMMdd_HHmmss}_pointCloud.ply");
//             using (StreamWriter writer = new StreamWriter(filePath))
//             {
//                 // PLYファイルヘッダの書き込み
//                 writer.WriteLine("ply");
//                 writer.WriteLine("format ascii 1.0");
//                 writer.WriteLine($"element vertex {_accumulatedPoints.Count}");
//                 writer.WriteLine("property float x");
//                 writer.WriteLine("property float y");
//                 writer.WriteLine("property float z");
//                 writer.WriteLine("property float intensity");
//                 writer.WriteLine("end_header");

//                 // 点群データの書き込み
//                 foreach (var point in _accumulatedPoints)
//                 {
//                     writer.WriteLine($"{point.position.x} {point.position.y} {point.position.z} {point.intensity}");
//                 }
//             }
//         }

//         protected override void OnSensorDestroy()
//         {
//             _jobHandle.Complete();
//             _noises.Dispose();
//             _directions.Dispose();
//             _raycastCommands.Dispose();
//             _raycastHits.Dispose();
//             base.OnSensorDestroy();
//         }

//         private void OnApplicationQuit()
//         {
//             // アプリケーション終了時に点群を保存
//             SaveAccumulatedPointsToPLY();
//             Application.Quit();
//         }
//     }
// }
// using System.Collections;
// using System.Collections.Generic;
// using UnityEngine;

// public class scan_correct : MonoBehaviour
// {
//     // Start is called before the first frame update
//     void Start()
//     {
        
//     }

//     // Update is called once per frame
//     void Update()
//     {
        
//     }
// }
