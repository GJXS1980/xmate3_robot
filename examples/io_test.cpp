/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * IO指令示例
 */

#include <cmath>
#include <iostream>
#include <functional>

#include "ini.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "control_tools.h"

using namespace xmate;

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
    robot.setMotorPower(1);
    robot.setDO(RCI::robot::DOSIGNAL::DO0_1,false);
    bool res = robot.getDI(RCI::robot::DISIGNAL::DI0_1);
    std::cout<<"res:"<<res<<std::endl;
}
