#include <cmath>
#include <iostream>
#include <functional>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "control_tools.h"

#include <ros/ros.h>
#include <ros/init.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/String.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
 
using namespace std;
using namespace xmate;
using namespace std;
using namespace Eigen;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

// 写死  机械臂连接
std::string ipaddr = "192.168.1.60";    // 机器人ip
uint16_t port = 1337;   //  机器人端口
xmate::Robot robot(ipaddr, port);

RCI::robot::RobotState robot_state; //  机器人状态

// 317.098098        355.734329    38.636231

const double PI = 3.14159;
std::array<double, 7> identify_ar_pose = {{(78.6055847 * PI / 180), (-14.9760269* PI / 180), (2.24453430 * PI / 180), (-59.1972610 * PI / 180), (3.67745018 * PI / 180), (-102.709705 * PI / 180), (-14.215536 * PI / 180)}};

std::array<double, 7> pubsh_cen_pose = {{(-7.8117462 * PI / 180), (-14.5854766* PI / 180), (0.30552978 * PI / 180), (-74.3494812 * PI / 180), (2.19893074 * PI / 180), (-85.5101108 * PI / 180), (-14.0703105 * PI / 180)}};

std::array<double,7> pubsh_pose = {{(-14.7492416 * PI / 180), (-7.7287033 * PI / 180), (3.25703460 * PI / 180), (-102.387483 * PI / 180), (2.40985107 * PI / 180), (-68.820299 * PI / 180), (-14.5463962 * PI / 180)}};

/*--------------------------------------------------------*/

bool test_DI(int di1){
    int timeout = 3;
    if(di1 == 0){
        RCI::robot::DISIGNAL DI_signal_0 = RCI::robot::DISIGNAL::DI1_0;
            bool DI_State = robot.getDI(DI_signal_0);
            if(!DI_State){
                while(DI_State != true && timeout>0){
                    DI_State = robot.getDI(DI_signal_0);
                    if(DI_State){
                        return true;
                    }
                    sleep(1.0);
                    timeout -=1;
                }
                return false;
            }
            else{
                return true;
            }

    }
    else if(di1 == 1){
        RCI::robot::DISIGNAL DI_signal_1 = RCI::robot::DISIGNAL::DI1_1;
            bool DI_State1 = robot.getDI(DI_signal_1);
            if(!DI_State1){
                while(DI_State1 != true && timeout>0){
                    DI_State1 = robot.getDI(DI_signal_1);
                    if(DI_State1){
                        return true;
                    }
                    sleep(1.0);
                    timeout -=1;
                }
                return false;
            }
            else{
                return true;
            }
    }
    else{
        return false;
    }
}

// do1_0 - 夹爪闭合  do1_1 开
// di1_0   闭合成功    di1_1 开成功 
// 参数０：闭合　　参数１：开
bool test_DO(int do1){

    if(do1 == 0){
        RCI::robot::DOSIGNAL DO_signal = RCI::robot::DOSIGNAL::DO1_0;
            robot.setDO(DO_signal,true);
            sleep(1.0);
            if(test_DI(do1)){
                sleep(1.0);
                robot.setDO(DO_signal,false);
            }else{
                robot.setDO(DO_signal,false);
            }
            return true;
    }
    else if(do1 == 1){
        RCI::robot::DOSIGNAL DO_signal = RCI::robot::DOSIGNAL::DO1_1;
            robot.setDO(DO_signal,true);
            sleep(1.0);
            if(test_DI(do1)){
                sleep(1.0);
                robot.setDO(DO_signal,false);
            }else{
                robot.setDO(DO_signal,false);
            }
            return true;
    }
    else{
        return false;
    }
}

//机械臂移动到识别位姿
bool robot_move_to_identify_ar_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, identify_ar_pose, robot);
    sleep(2);
    return true;
}

// 机械臂移动到pubsh的中间位姿
bool robot_move_to_pubsh_cen_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, pubsh_cen_pose, robot);
    sleep(2.0);
    return true;
}

// 机械臂移动到pubsh位姿
bool robot_move_to_pubsh_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, pubsh_pose, robot);
    sleep(2.0);
    return true;
}

//放置动作
bool robot_push_action(){
    cart_pos current_pose, end_pose;

    /*robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[7] -= 0.002;
    MOVEL(0.1, current_pose, end_pose, robot);
    sleep(1.0);*/

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= 0.026636231;
    MOVEL(0.1, current_pose, end_pose, robot);
    sleep(2.0);
    test_DO(0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.038636231;
    MOVEL(0.1, current_pose, end_pose, robot);
    sleep(2.0);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] -= 0.036636231;
    MOVEL(0.1, current_pose, end_pose, robot);
    sleep(2.0);
    test_DO(1);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.038636231;
    MOVEL(0.1, current_pose, end_pose, robot);
    sleep(2.0);

    return true;
}


// bool robot_move_to_get_pose(){
//     std::array<double,7> q_init;
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState().q;
//     MOVEJ(0.2, q_init, get_pose, robot);
//     sleep(2.0);
//     return true;
// }

int main(int argc, char *argv[]) {
    robot.automaticErrorRecovery();

    /*for(int i=0;i<3;i++){
        test_DO(0);
        sleep(1.0);
        test_DO(1);
        sleep(1.0);
    }
    */
    robot_move_to_identify_ar_pose();
    robot_move_to_pubsh_cen_pose();
    robot_move_to_pubsh_pose();
    robot_push_action(); 
    
    return 0;

}
