//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Control
{
    [Serializable]
    public class JointTrajectoryGoal : Message
    {
        public const string k_RosMessageName = "control_msgs/JointTrajectory";
        public override string RosMessageName => k_RosMessageName;

        public Trajectory.JointTrajectoryMsg trajectory;

        public JointTrajectoryGoal()
        {
            this.trajectory = new Trajectory.JointTrajectoryMsg();
        }

        public JointTrajectoryGoal(Trajectory.JointTrajectoryMsg trajectory)
        {
            this.trajectory = trajectory;
        }

        public static JointTrajectoryGoal Deserialize(MessageDeserializer deserializer) => new JointTrajectoryGoal(deserializer);

        private JointTrajectoryGoal(MessageDeserializer deserializer)
        {
            this.trajectory = Trajectory.JointTrajectoryMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.trajectory);
        }

        public override string ToString()
        {
            return "JointTrajectoryGoal: " +
            "\ntrajectory: " + trajectory.ToString();
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
