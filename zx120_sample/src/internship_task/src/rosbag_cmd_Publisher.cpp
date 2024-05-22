#include <ros/ros.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sstream>

const static int NUM_AXIS = 4;
const static int SWING = 0, BOOM = 1, ARM = 2, BUCKET = 3;
bool flag = false;

std::vector<double> joint_pos(NUM_AXIS);//分解したcmdを格納するvector
std::vector<double> joint_return_pos(NUM_AXIS);//戻り値のcmdを格納するvector
std::vector<std::string> axises = {"Swing","boom","arm","bucket"};//文字列配列

void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)//rosbagから関節角度を取得し格納するコールバック関数
{
    for(int i = 0; i < msg->name.size();i++)
    {
        if (msg->name[i] == "swing_joint")
        {
            joint_pos[SWING] = msg->position[i];
        }
        else if (msg->name[i] == "boom_joint")
        {
            joint_pos[BOOM] = msg->position[i];
        }
        else if(msg->name[i] == "arm_joint")
        {
            joint_pos[ARM] = msg->position[i];
        }
        else if (msg->name[i] == "bucket_joint")
        {
            joint_pos[BUCKET] = msg->position[i]; 
        }
    }
    flag = true;
}

void JointStateDiffCallback(const sensor_msgs::JointState::ConstPtr &msg){//unityから関節角度を取得し格納するコールバック関数
    for(int i = 0;i<msg->name.size();i++){
        if (msg->name[i] == "swing_joint"){
            joint_return_pos[i] = msg->position[i];
        }
        else if (msg->name[i] == "boom_joint")
        {
            joint_return_pos[i] = msg->position[i];
        }
        else if (msg->name[i] == "arm_joint")
        {
            joint_return_pos[i] = msg->position[i];
        }
        else if (msg->name[i] == "bucket_joint")
        {
            joint_return_pos[i] = msg->position[i];
        }
    }
}

void timeoutCallback(const ros::TimerEvent& event){
    if (!flag)
    {
        ros::shutdown();
    }
    
}

int main(int argc,char **argv)
{
    std::vector<std_msgs::Float64> joint_cmd(NUM_AXIS);
    std::vector<std_msgs::Float64> return_cmd(NUM_AXIS);
    std::vector<double> joint_Diff(NUM_AXIS);
    std::vector<double> acuum_joint_Diff(NUM_AXIS);
    std::vector<double> Diff_vec(NUM_AXIS,0.0);
    double Diff = 0.0;

    ros::init(argc,argv,"rosbag_cmd_Publisher");
    ros::NodeHandle n;

    ros::Publisher swing_cmd_pub = n.advertise<std_msgs::Float64>("zx120/swing/cmd",100);
    ros::Publisher boom_cmd_pub = n.advertise<std_msgs::Float64>("zx120/boom/cmd",100);
    ros::Publisher arm_cmd_pub = n.advertise<std_msgs::Float64>("zx120/arm/cmd",100);
    ros::Publisher bucket_cmd_pub = n.advertise<std_msgs::Float64>("zx120/bucket/cmd",100);

    ros::Subscriber joint_state_sub  = n.subscribe("zx120/imubased/joint_states",100,JointStateCallback);
    ros::Subscriber joint_state_diff_sub = n.subscribe("zx120/joint_states",100,JointStateDiffCallback);

    ros::Timer timer = n.createTimer(ros::Duration(5.0),timeoutCallback);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {
    joint_cmd[SWING].data = -0.012806;//コールバックが行われるまではrosbagの一番最初の関節角データを入れておく
    joint_cmd[BOOM].data = -0.976184;
    joint_cmd[ARM].data = 2.830586;
    joint_cmd[BUCKET].data = -0.118901;

    if (flag)//もしコールバックが実行されrosbagから関節角度を取得できたらjoint_cmd配列に格納する
    {
    joint_cmd[SWING].data = joint_pos[SWING];
    joint_cmd[BOOM].data = joint_pos[BOOM];
    joint_cmd[ARM].data = joint_pos[ARM];
    joint_cmd[BUCKET].data = joint_pos[BUCKET];        
    }

    return_cmd[SWING].data = joint_return_pos[SWING];//unityから取得した関節角度をjoint_return_pos配列に格納する
    return_cmd[BOOM].data = joint_return_pos[BOOM];
    return_cmd[ARM].data = joint_return_pos[ARM];
    return_cmd[BUCKET].data = joint_return_pos[BUCKET];

    ROS_INFO("Swing=%f,Boom=%f,Arm=%f,Bucket=%f",joint_cmd[SWING].data,joint_cmd[BOOM].data,joint_cmd[ARM].data,joint_cmd[BUCKET].data);
    ROS_INFO("Now Sim_AXIS Swing=%f,Boom=%f,Arm=%f,Bucket=%f",return_cmd[SWING].data,return_cmd[BOOM].data,return_cmd[ARM].data,joint_cmd[BUCKET].data);

    swing_cmd_pub.publish(joint_cmd[SWING]);//rosbagの関節角を保存したjoint_cmdをパブリッシュ
    boom_cmd_pub.publish(joint_cmd[BOOM]);
    arm_cmd_pub.publish(joint_cmd[ARM]);
    bucket_cmd_pub.publish(joint_cmd[BUCKET]);

    if (flag)
    {
    for (int i = 0; i < NUM_AXIS; i++) {
        joint_Diff[i] = joint_cmd[i].data - joint_return_pos[i];//rosbagから取得した関節角度(rad)-シミュレータから取得した関節角度(rad)
        ROS_INFO("joint_Diff[%d]:%f", i, joint_Diff[i]);

        double squared_value = pow(joint_Diff[i], 2.0);
        ROS_INFO("squared_value:%f", squared_value);

        Diff_vec[i] += squared_value;
        ROS_INFO("Updated Diff_vec[%d]:%f", i, Diff_vec[i]);

        acuum_joint_Diff[i] = sqrt(Diff_vec[i]);
        ROS_INFO("acuum_joint_Diff[%d]:%f (should be %f)", i, acuum_joint_Diff[i], sqrt(Diff_vec[i]));
        }
    }
    for (size_t i = 0; i < axises.size(); i++)
    {
        ROS_INFO("%s_acuum_Diff= %f",axises[i].c_str(),acuum_joint_Diff[i]);
    }
    std::cout << "---------------------------------------------------" << std::endl;
    flag = false;
    ros::spinOnce();
    loop_rate.sleep();
    }
    return 0;
}