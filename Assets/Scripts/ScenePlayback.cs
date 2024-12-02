using System.Collections;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;//これ使うとROSとの通信が可能
using RosMessageTypes.Std;

public class ScenePlayback : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("unity_scene_playback");
        StartCoroutine(StartRosExecutionAfterDelay(3f));
    }

    IEnumerator StartRosExecutionAfterDelay(float delay)
    {
        yield return new WaitForSeconds(delay);
        StringMsg msg = new StringMsg("start_ros_execution");
        ros.Publish("unity_scene_playback", msg);
    }
}

