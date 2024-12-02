// using UnityEngine;

// using Unity.Burst;
// using Unity.Collections;
// using Unity.Jobs;
// using Unity.Mathematics;

// using UnitySensors.Data.PointCloud;

// namespace UnitySensors.Sensor.LiDAR
// {
//     [BurstCompile]
//     public struct IRaycastHitsToPointsJob : IJobParallelFor
//     {
//         [ReadOnly]
//         public float minRange;
//         [ReadOnly]
//         public float minRange_sqr;
//         [ReadOnly]
//         public float maxRange;
//         [ReadOnly]
//         public float maxIntensity;
//         [ReadOnly]
//         public float3 sensorPosition;
//         [ReadOnly]
//         public float3 scanRange;
//         [ReadOnly, NativeDisableParallelForRestriction]
//         public NativeArray<float3> directions;
//         [ReadOnly]
//         public int indexOffset;
//         [ReadOnly]
//         public NativeArray<RaycastHit> raycastHits;
//         [ReadOnly]
//         public NativeArray<float> noises;

//         public NativeArray<PointXYZI> points;

//         public void Execute(int index)
//         {
//             int idx = (index + indexOffset) % directions.Length;
//             float3 direction = directions[idx];
//             RaycastHit hit = raycastHits[index];

//             if (hit.distance > 0 && hit.distance < maxRange)
//             {
//                 float3 hitPoint = (float3)hit.point + direction * noises[index];
//                 float3 relativePosition = hitPoint - sensorPosition;

//                 // スキャン範囲内にあるかチェック
//                 if (math.abs(relativePosition.x) <= scanRange.x && math.abs(relativePosition.y) <= scanRange.y && math.abs(relativePosition.z) <= scanRange.z)
//                 {
//                     float intensity = maxIntensity * math.exp(-hit.distance / maxRange);
//                     points[index] = new PointXYZI()
//                     {
//                         position = hitPoint,
//                         intensity = intensity
//                     };
//                 }
//                 else
//                 {
//                     points[index] = new PointXYZI { position = float3.zero, intensity = 0 }; // 範囲外の点は初期化
//                 }
//             }
//             else
//             {
//                 points[index] = new PointXYZI { position = float3.zero, intensity = 0 }; // 範囲外の点は初期化
//             }
//         }
//     }
// }
