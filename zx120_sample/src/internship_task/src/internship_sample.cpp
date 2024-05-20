#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <sstream>

const static int NUM_AXIS = 4;                               //ショベルの動く軸(swing, boom, arm, bucket)の数
const static int SWING = 0, BOOM = 1, ARM = 2, BUCKET = 3;   //各軸のIDの定義
const static int MODE0 = 0, MODE1 = 1, MODE2 = 2, MODE3 = 3,MODE4 = 4,MODE5 = 5,MODE6 = 6,MODE7 = 7; //各モードIDの定義
const static double SRES_ERROR = 0.3;                        //角度偏差の許容値（ここはビルドしなおさなくて済むよう、rosparamで設定できるのが望ましいが）
/*上の角度偏差の許容値と書いてあるのが次のMODEに行くことのできる許容値*/

std::vector<double> joint_pos(NUM_AXIS); //各関節の現在角度

/* subscribe対象のメッセージを受信する度に一度実行される関数 */
/*ここで現在の関節角度を入力していると思う*/ 
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    //ここにはjoint_stateメッセージを受信した時に行う処理を記述する
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (msg->name[i] == "swing_joint")
        {
            joint_pos[SWING] = msg->position[i];
        }
        else if (msg->name[i] == "boom_joint")
        {
            joint_pos[BOOM] = msg->position[i];
        }
        else if (msg->name[i] == "arm_joint")
        {
            joint_pos[ARM] = msg->position[i];
        }
        else if (msg->name[i] == "bucket_joint")
        {
            joint_pos[BUCKET] = msg->position[i];
        }
    }
}

int main(int argc, char **argv)
{
    std::vector<std_msgs::Float64> joint_cmd(NUM_AXIS); //各関節の目標角度
    std::vector<double> joint_error(NUM_AXIS); //各関節の目標角度と現在角度の差分
    double error_norm = 0.0;

    ros::init(argc, argv, "sample_node");
    ros::NodeHandle n;

    /* Publisherの宣言 */
    ros::Publisher swing_cmd_pub = n.advertise<std_msgs::Float64>("zx120/swing/cmd", 100);
    ros::Publisher boom_cmd_pub = n.advertise<std_msgs::Float64>("zx120/boom/cmd", 100);
    ros::Publisher arm_cmd_pub = n.advertise<std_msgs::Float64>("zx120/arm/cmd", 100);
    ros::Publisher bucket_cmd_pub = n.advertise<std_msgs::Float64>("zx120/bucket/cmd", 100);

    /* Subscriberの宣言 */
    ros::Subscriber joint_state_sub = n.subscribe("zx120/joint_states", 100, jointStateCallback);

    /* While loopが回る周期 [Hz] */
    ros::Rate loop_rate(20);

    int count = 0;
    int mode = MODE0; //最初の状態機械をMODE0とする

    ros::Time mode_start_time = ros::Time::now();
    double mode_wait_time = 4.0; // モードごとの待ち時間

    /*大きくすると手前に回る*/
    joint_cmd[SWING].data = 0.0;
    /*大きくすると下に下がる*/
    joint_cmd[BOOM].data = -1.0;
    /*大きくすると上に上がる*/
    joint_cmd[ARM].data = 1.0;
    /*大きくすると建機側に曲がる*/
    joint_cmd[BUCKET].data = 0.0;

    while (ros::ok())
    {
        // ROS_INFO("msg=%f", joint_pos_cmd[SWING].data);

        /* 各関節誤差のnorm値を計算する */
        for (int i = 0; i < NUM_AXIS; i++)
        {
            joint_error[i] = joint_cmd[i].data - joint_pos[i];
            error_norm += pow(joint_error[i], 2.0);
        }
        /*各関節誤差の二乗をルートしている→それがerror_norm*/
        error_norm = sqrt(error_norm);

        switch (mode)
        {
        case MODE0:
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = -1.0;
            joint_cmd[ARM].data = 1.0;
            joint_cmd[BUCKET].data = 0.0;

            ROS_INFO("error_norm=%f, mode=%d", error_norm, mode);

            /* 状態機械の遷移条件と遷移先の状態機械を記述する */
            /*多分誤差が小さくなったら次のフェーズ(MODE)に移動する*/
            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE1;
                mode_start_time = ros::Time::now(); // モード1に遷移した時点で時刻を更新
            }
            break;

        case MODE1://刃先位置決定(土に強めに差し込んでいます)
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = -0.175;
            joint_cmd[ARM].data = 1.0;
            joint_cmd[BUCKET].data = 0.0;

            /* 状態機械の遷移条件と遷移先の状態機械を記述する */
            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE2;
                mode_start_time = ros::Time::now(); // モード2に遷移した時点で時刻を更新
            }
            break;

        case MODE2://掘削(一度深くまでバケットを持っていっています)
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = 0.24;
            joint_cmd[ARM].data = 1.5;
            joint_cmd[BUCKET].data = 0.5;

            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE3;
                mode_start_time = ros::Time::now(); // モード3に遷移した時点で時刻を更新
            }
            break;

        case MODE3://掘削(MODE2で深く入れたバケットを上に持ち上げることで粒子を取得します。)
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = 0.1;
            joint_cmd[ARM].data = 1.75;
            joint_cmd[BUCKET].data = 1.0;

            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE4;
                mode_start_time = ros::Time::now(); // モード3に遷移した時点で時刻を更新
            }
            // ROS_INFO("Sample motion completes!");
            break;

        case MODE4://若干持ち上げた
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = -0.2;
            joint_cmd[ARM].data = 1.75;
            joint_cmd[BUCKET].data = 2.0;

            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE5;
                mode_start_time = ros::Time::now(); // モード3に遷移した時点で時刻を更新
            }
            // ROS_INFO("Sample motion completes!");
            break;

        case MODE5://一度上までboomを持っていった
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = -0.5;
            joint_cmd[ARM].data = 1.25;
            joint_cmd[BUCKET].data = 2.0;

            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE6;
                mode_start_time = ros::Time::now(); // モード3に遷移した時点で時刻を更新
            }
            break;

        case MODE6://取得した土を落としてみた
            joint_cmd[SWING].data = 0.0;
            joint_cmd[BOOM].data = -0.5;
            joint_cmd[ARM].data = 1.25;
            joint_cmd[BUCKET].data = 0.0;

            if (error_norm < SRES_ERROR && (ros::Time::now() - mode_start_time).toSec() > mode_wait_time)
            {
                mode = MODE7;
                mode_start_time = ros::Time::now(); // モード3に遷移した時点で時刻を更新
            }
            break;
        
        case MODE7:
            break;
        default:
            ROS_WARN("mode has undefined value!");
            break;
        }

        /* 各関節の目標角度をROSメッセージとしてPublishする処理 */
        swing_cmd_pub.publish(joint_cmd[SWING]);
        boom_cmd_pub.publish(joint_cmd[BOOM]);
        arm_cmd_pub.publish(joint_cmd[ARM]);
        bucket_cmd_pub.publish(joint_cmd[BUCKET]);

        /* 表示させたい情報をROS_INFO関数でターミナル上へテキスト出力する */
        ROS_INFO("error_norm=%f, mode=%d", error_norm, mode);

        ros::spinOnce();
        loop_rate.sleep();
        error_norm = 0.0; /* error_normのクリア */
        ++count;
    }

    return 0; //プログラム終了
}
