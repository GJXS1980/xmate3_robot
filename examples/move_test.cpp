/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * MOVE指令示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "control_tools.h"

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.3.41";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    xmate::Robot robot(ipaddr, port);
    sleep(1);
    // robot.setMotorPower(1);
    robot.automaticErrorRecovery();

    const double PI = 3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);

    RCI::robot::RobotState robot_state = robot.receiveRobotState();
    cart_pos pos_start,pos_end,pos_middle;
    pos_start.pos = robot_state.toolTobase_pos_m;

    Eigen::Matrix3d rot_start,rot_change,rot_end;
    Eigen::Vector3d trans_start,trans_change,trans_end;
    ArrayTo(pos_start.pos,rot_start,trans_start);
    Eigen::Vector3d euler(-0.1,-0.1,-0.1);
    EulerToMatrix(euler,rot_change);
    rot_end = rot_start;
    //rot_end = rot_start*rot_change;
    trans_end = trans_start;
    trans_end[1] -= 0.2;
    ToArray(rot_end,trans_end,pos_end.pos);
    pos_middle = pos_start;
    pos_middle.pos[7] -= 0.1;
    pos_middle.pos[3] -= 0.1;
    // pos_start.psi_valid = true;
    // pos_start.psi = robot_state.psi_m;
    // pos_end.psi_valid = true;
    // pos_end.psi = robot_state.psi_m;
    MOVEL(0.2, pos_start, pos_end,robot);
    MOVEL(0.2, pos_end, pos_start,robot);
    MOVEC(0.2,pos_start,pos_middle,pos_end,robot);
    MOVEC(0.2,pos_end,pos_middle,pos_start,robot);

    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition, RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    std::array<double, 7> init_position;
    static bool init = true;
    double time = 0;

    JointControl joint_position_callback;
    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        time += 0.001; 
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

        JointPositions output = {{init_position[0] + delta_angle, init_position[1] + delta_angle,
                                            init_position[2] + delta_angle, init_position[3] - delta_angle,
                                            init_position[4] + delta_angle, init_position[5] - delta_angle,
                                            init_position[6] + delta_angle}}; 
        if(time>20){
            std::cout<<"运动结束："<<std::endl;
            return MotionFinished(output);
        }
        return output;        
    };
    robot.Control(joint_position_callback);
    return 0;
}
