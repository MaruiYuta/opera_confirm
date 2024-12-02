// using System.Collections;
// using UnityEngine;

// public class RotateObject : MonoBehaviour
// {
//     // 回転速度を指定します（度/秒）
//     public float rotationSpeed = 20.0f;  // 例えば90度/秒
//     public float delayTime = 52.0f;     // 180秒の遅延時間

//     // 回転した角度を追跡するための変数
//     private float totalRotation = 0.0f;

//     void Start()
//     {
//         // 初期状態でスクリプトを無効化
//         enabled = false;

//         // 180秒後に回転を開始するコルーチンを開始
//         StartCoroutine(DelayedRotationStart());
//     }

//     private IEnumerator DelayedRotationStart()
//     {
//         // 135秒間待つ
//         yield return new WaitForSeconds(delayTime);

//         // 回転を開始するためにスクリプトを有効化
//         enabled = true;
//     }

//     void Update()
//     {
//         // 今フレームでの回転量を計算
//         float rotationThisFrame = rotationSpeed * Time.deltaTime;

//         // 合計回転量を更新
//         totalRotation += rotationThisFrame;

//         // 回転量が360度を超えないようにする
//         if (totalRotation >= 180.0f)
//         {
//             rotationThisFrame -= (totalRotation - 180.0f);
//             totalRotation = 180.0f;  // 正確に360度に設定
//         }

//         // オブジェクトを回転させる
//         transform.Rotate(Vector3.right * rotationThisFrame);

//         // 合計回転量が360度に達したら回転を停止
//         if (totalRotation >= 180.0f)
//         {
//             enabled = false;  // このスクリプトを無効化して回転を停止
//         }
//     }
// }

// using System.Collections;
// using UnityEngine;

// public class ObjectMover : MonoBehaviour
// {
//     public float moveSpeed = 0.7f;  // 移動速度
//     public Vector3 moveDirection = Vector3.forward;  // 移動方向
//     public float moveDuration = 9.0f;  // 移動を行う時間
//     public float executionDelay = 38.0f;  // 移動が開始されるまでの遅延時間

//     private bool _isMoving = false;
//     private float _startTime;

//     void Start()
//     {
//         // 指定時間後に移動を開始するコルーチンを実行
//         StartCoroutine(DelayedStartMoving());
//     }

//     private IEnumerator DelayedStartMoving()
//     {
//         // 指定された遅延時間を待つ
//         yield return new WaitForSeconds(executionDelay);
//         _isMoving = true;
//         _startTime = Time.time;
//         Debug.Log("Object movement started.");
//     }

//     void Update()
//     {
//         // 指定された移動時間が経過するまでオブジェクトを移動
//         if (_isMoving && (Time.time - _startTime) <= moveDuration)
//         {
//             transform.Translate(moveDirection * moveSpeed * Time.deltaTime, Space.World);
//         }
//         else if (_isMoving)
//         {
//             // 移動を停止
//             _isMoving = false;
//             Debug.Log("Object movement stopped.");
//         }
//     }
// }

using System.Collections;
using UnityEngine;

public class ObjectMover : MonoBehaviour
{
    public float moveSpeed = 0.7f;  // 移動速度
    public Vector3[] moveDirections;  // 複数の移動方向
    public float[] moveDurations;  // 各移動動作の時間
    public float executionDelay = 38.0f;  // 移動が開始されるまでの遅延時間

    private bool _isMoving = false;
    private float _startTime;
    private int _currentMoveIndex = 0;  // 現在の移動インデックス

    void Start()
    {
        // コルーチンで指定時間後に移動を開始
        StartCoroutine(DelayedStartMoving());
    }

    private IEnumerator DelayedStartMoving()
    {
        // 指定された遅延時間を待つ
        yield return new WaitForSeconds(executionDelay);
        StartNextMove();
    }

    private void StartNextMove()
    {
        // 次の移動を開始
        if (_currentMoveIndex < moveDirections.Length && _currentMoveIndex < moveDurations.Length)
        {
            _isMoving = true;
            _startTime = Time.time;
            Debug.Log("Object movement started: " + moveDirections[_currentMoveIndex] + " for " + moveDurations[_currentMoveIndex] + " seconds.");
        }
        else
        {
            Debug.Log("All movements completed.");
        }
    }

    void Update()
    {
        if (_isMoving)
        {
            // 現在の移動が指定時間内かどうかをチェック
            if ((Time.time - _startTime) <= moveDurations[_currentMoveIndex])
            {
                // 指定された方向にオブジェクトを移動
                transform.Translate(moveDirections[_currentMoveIndex] * moveSpeed * Time.deltaTime, Space.World);
            }
            else
            {
                // 移動終了後、次の動作を開始
                _isMoving = false;
                _currentMoveIndex++;
                StartNextMove();
            }
        }
    }
}
