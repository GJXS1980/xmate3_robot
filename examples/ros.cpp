#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "model.h"
#include "move.h"

#include <ros/ros.h>
#include <ros/init.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include "RobotStateInfo.h"

//#include "ros_msg/ros.h"

using namespace std;
using namespace xmate;

using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[]) {

    //ros节点初始化
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle core;
    ros_msg::RobotStateInfo m_robot_state_msg;
    m_robot_state_msg.JointState.resize(7);
    m_robot_state_msg.CartPose.resize(6);
    ros::Publisher m_robot_state_pub = core.advertise<ros_msg::RobotStateInfo>("/robot_state_info",10,false);  

    //机器人ip地址设置（目前写死）
    std::string ipaddr = "192.168.1.160";
    uint16_t port = 1337;
    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port);
    //防止网络连接失败
    
    sleep(1);
    
    std::cout<<"00000机械臂当前状态:"<< robot.receiveRobotState().robot_mode<<std::endl;
    robot.automaticErrorRecovery();
    //robot.setMotorPower(1);
    sleep(2);

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    //实时获取机器人当前关节角
    
    q_init = robot.receiveRobotState().q;
    std::cout<<"11111机械臂当前状态:"<< robot.receiveRobotState().robot_mode<<std::endl;
    //MOVEJ指令到q_drag点位姿
    MOVEJ(0.2,q_init,q_drag,robot);
    std::cout<<"------------run here---------------"<<std::endl;
 
    q_init = robot.receiveRobotState().q;

    robot.setMotorPower(0);
    sleep(2);

    xmate::XmateModel model(&robot,xmate::XmateType::XMATE7);

    /*设置TCP坐标系/法兰坐标系，不设置的话默认的是法兰坐标系，
    * 如需设置TCP坐标系只需要把TCP坐标系相对于法兰坐标系的参数通过setCoor设置
    */
    robot.setCoor({{1,0,0,0,
                0,1,0,0,
                0,0,1,0.2,
                0,0,0,1}});
    //规划一条回零点的关节空间运动轨迹
    std::array<double, 7> q_end = {{0, 0, 0, 0, 0, 0, 0}};
    JointMotionGenerator joint_s(0.1, q_end);

    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    std::array<double, 7> init_position;
    std::array<double, 7> joint_delta;
    //定义关节空间位置变量output{}
    JointPositions output{};
    static bool init = true;
    double time = 0;

    JointControl joint_position_callback;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
    //ros创建时间节点
    ros::Time current = ros::Time::now();
    m_robot_state_msg.header.stamp = current;
    //将实时的机器人关节角赋值给ros消息JointState变量
    for(unsigned i=0;i<7;i++){
        m_robot_state_msg.JointState[i] = robot_state.q[i];
    }
    //调用FK接口获取笛卡尔空间角
    std::array<double, 16> q_cartpos;
    q_cartpos = model.GetCartPose(robot_state.q);
    //调用IK接口GetJointPos获取关节空间角,第一个参数为输入关节角，第二个参数为机器人臂角一般为0，第三个参数为初始关节角度一般可获取当前位置值
    std::array<double,7> q_jntpos,q_current_jntpos;
    q_current_jntpos = robot_state.q;
    q_jntpos = model.GetJointPos(q_cartpos, 0.0, q_current_jntpos);
    //定义Eigen变量，将robotstate中的笛卡尔空间位姿通过Eigen转换成笛卡尔空间xyz rpy值
    Eigen::Matrix3d cart_rot;
    cart_rot << robot_state.toolTobase_pos_m[0], robot_state.toolTobase_pos_m[1], robot_state.toolTobase_pos_m[2], robot_state.toolTobase_pos_m[4], robot_state.toolTobase_pos_m[5], 
                 robot_state.toolTobase_pos_m[6], robot_state.toolTobase_pos_m[8], robot_state.toolTobase_pos_m[9], robot_state.toolTobase_pos_m[10];
    Eigen::Vector3d euler = cart_rot.eulerAngles(2,1,0);
    Eigen::VectorXd cart_pos(6);
    cart_pos<<robot_state.toolTobase_pos_m[3],robot_state.toolTobase_pos_m[7],robot_state.toolTobase_pos_m[11],euler[0],euler[1],euler[2];
    //将笛卡尔位姿赋值给ros消息CartPos变量
    for(unsigned i=0;i<6;i++){
        m_robot_state_msg.CartPose[i] = cart_pos[i];
    }
    //发布ros消息
    m_robot_state_pub.publish(m_robot_state_msg);
    //刷新周期为1ms，读入初始关节角给init——position
        time += 0.001; 
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }

        joint_s.calculateSynchronizedValues_joint(init_position);
        //实时获取每个周期的关节角赋值给output.q
        if (joint_s.calculateDesiredValues_joint(time, joint_delta) == false) {
            output.q = {{init_position[0] + joint_delta[0], init_position[1] + joint_delta[1],
                                    init_position[2] + joint_delta[2], init_position[3] + joint_delta[3],
                                    init_position[4] + joint_delta[4], init_position[5] + joint_delta[5],
                                    init_position[6] + joint_delta[6]}};
            
        }else{
            std::cout<<"运动结束："<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };
    //给机器人发点的操作是通过joint_position_callback回调来执行；如果是笛卡尔空间则是cartesian_position_callback来实现
    robot.Control(joint_position_callback);

    return 0;


  
}


