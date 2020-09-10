/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 关节空间位置运动示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"

using namespace xmate;
int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.3.41";
    uint16_t port = 1337;
    std::string file = "../../xmate.ini";
    INIParser ini;
    double T = 2.5;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port);
    //防止网络连接失败
    sleep(1);
    robot.setMotorPower(1);

    std::string joint_callback = "joint_callback";
    std::string cart_callback = "cart_callback";

    robot.setFilters(10, 10, 40);
    robot.setCollisionBehavior({{50,50,50,50,50,50,50}});

    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    uint64_t init_time;
    std::array<double, 7> init_position;
    std::array<double, 16> init_position_cart;
    double time = 0;

    auto cartesian_position_callback = [&](char *p) {
        RCI::robot::RobotState *robot_state = reinterpret_cast<RCI::robot::RobotState *>(p);
        static bool bfirst = true;
        if (bfirst) {
            //保存一个运动开始的时间initial_time
            //保存一个运动开始的位置initial_position
            init_time = robot_state->message_id;
            init_position_cart = robot_state->toolTobase_pos_m;
            bfirst = false;
        }
        time += 0.001;
        if (time >= 0) {
            try {
                robot.throwOnMotionError(*robot_state);
                if (time > 100) {
                    robot.stopMove();
                }
                constexpr double kRadius = 0.1;
                double angle = M_PI / 4 * (1 - std::cos(M_PI / 1 * time));
                double delta_x = kRadius * std::sin(angle);
                double delta_z = kRadius * (std::cos(angle) - 1);

                std::array<double, 16> new_pose = init_position_cart;
                new_pose[11] += delta_z;

                RCI::robot::MotionCommand motion_command{};
                RCI::robot::TorqueCommand control_comamnd{};
                RCI::robot::RobotCommand robot_command{};

                robot_command.message_id = robot_state->message_id;
                motion_command.toolTobase_pos_c = new_pose;
                robot_command.motion = motion_command;
                robot_command.torque = control_comamnd;
                //避免出现两个运动转换时突然等于0的情况;
                robot_command.motion.q_c = robot_state->q;

                robot.sendRobotCommand(robot_command, *robot_state);
                robot.log(robot_command, *robot_state);
            } catch (xmate::ControlException &e) {
                std::cout << e.what() << std::endl;
                std::string file_name = "data_error";
                xmate::writeLogToFile(e.log, file_name);
                robot.stopMove();
                exit(0);
            }
        }
    };

    auto joint_position_callback = [&](char *p) {
        RCI::robot::RobotState *robot_state = reinterpret_cast<RCI::robot::RobotState *>(p);
        static bool bfirst = true;
        if (bfirst) {
            // RobotState中的message_id即时间，单位是ns
            init_time = robot_state->message_id;
            init_position = robot_state->q;
            bfirst = false;
        }
        //用户规划用的时间
        time += 0.001;
        if (time >= 0) {
            try {
                robot.throwOnMotionError(*robot_state);
                double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI / T * time));
                std::array<double, 7> output = {{init_position[0] + delta_angle, init_position[1] + delta_angle,
                                                 init_position[2] + delta_angle, init_position[3] - delta_angle,
                                                 init_position[4] + delta_angle, init_position[5] - delta_angle,
                                                 init_position[6] + delta_angle}};

                RCI::robot::MotionCommand motion_command{};
                RCI::robot::TorqueCommand control_comamnd{};
                RCI::robot::RobotCommand robot_command{};

                robot_command.message_id = robot_state->message_id;
                motion_command.q_c = output;
                robot_command.motion = motion_command;
                robot_command.torque = control_comamnd;
                robot_command.motion.toolTobase_pos_c = robot_state->toolTobase_pos_m;

                robot.sendRobotCommand(robot_command, *robot_state);
                robot.log(robot_command, *robot_state);
                if (time > 20) {
                    std::cout << "用户规划停止，发送stopmove" << std::endl;
                    robot.stopMove();
                    time = 0;
                    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kCartesianPosition,
                                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kCartesianPosition);
                    robot.SetCallback(cartesian_position_callback, cart_callback);
                    robot.startUdp();
                }
            } catch (xmate::ControlException &e) {
                std::cout << "抓住异常，停止，发送stopmove" << std::endl;
                std::cout << e.what() << std::endl;
                std::string file_name = "data_error";
                xmate::writeLogToFile(e.log, file_name);
                robot.stopMove();
            }
        }
    };

    robot.SetCallback(joint_position_callback, joint_callback);
    robot.startUdp();
    std::string strcmd;
    do {
        std::cin >> strcmd;
    } while (strcmd != "exit" && strcmd != "quit");
    return 0;
}
