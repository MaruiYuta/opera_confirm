//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    [Serializable]
    public class SingleJointPositionResult : Message
    {
        public const string k_RosMessageName = "control_msgs/SingleJointPosition";
        public override string RosMessageName => k_RosMessageName;


        public SingleJointPositionResult()
        {
        }
        public static SingleJointPositionResult Deserialize(MessageDeserializer deserializer) => new SingleJointPositionResult(deserializer);

        private SingleJointPositionResult(MessageDeserializer deserializer)
        {
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
        }

        public override string ToString()
        {
            return "SingleJointPositionResult: ";
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
