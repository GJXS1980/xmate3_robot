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

const double PI = 3.14159;

// ar码pose
geometry_msgs::PoseStamped ar_traget_pose;  //  ar码的时间戳

// ar码ID
int ar_marker_id = 2;   //  ar码id

double robot_angle = 0.0;

std_msgs::String target_msg;

bool ready_to_grap = false; //  抓取的flag

int pubsh_time = 0;

bool ready_roation = false;
bool roation_flag = false;
int identify_time = 0;

bool move_again = false;
int identify_move_again = 0;

//机械臂起始位姿
std::array<double, 7> robot_start_pose = {{0,0,PI/2,PI/2,0,PI/2,0}};

//机械臂识别位姿
//std::array<double, 7> identify_ar_pose = {{0, -PI/18, 0, -PI/2.4, 0,-PI/2, 0}};
//std::array<double, 7> identify_ar_pose = {{(-85.1196945 * PI / 180), (-17.577809* PI / 180), (-4.2781036 * PI / 180), (-62.565518 * PI / 180), (8.35158348 * PI / 180), (-91.5974979 * PI / 180), (-0.21751213 * PI / 180)}};
std::array<double, 7> identify_ar_pose = {{(-105.440336 * PI / 180), (-8.28760528* PI / 180), (9.48256072 * PI / 180), (-71.7210983 * PI / 180), (0.74969673 * PI / 180), (-97.2024135 * PI / 180), (-16.9767436 * PI / 180)}};

//机械臂抓取位姿
//std::array<double, 7> grab_pose = {{0, 0, 0, PI/2, 0,PI/2, 0}};
std::array<double,7> grab_pose = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};

//fangzhi x -  z+ 531.77845 - 446.70611 放置位置

std::array<double, 7> pubsh_cen_pose = {{(-7.8117462 * PI / 180), (-14.5854766* PI / 180), (0.30552978 * PI / 180), (-74.3494812 * PI / 180), (2.19893074 * PI / 180), (-85.5101108 * PI / 180), (-14.0703105 * PI / 180)}};

std::array<double,7> pubsh_pose = {{(-14.7492416 * PI / 180), (-7.7287033 * PI / 180), (3.25703460 * PI / 180), (-102.387483 * PI / 180), (2.40985107 * PI / 180), (-68.820299 * PI / 180), (-14.5463962 * PI / 180)}};

/*--------------------------------------------------------*/

class ARPose{
public:
  int count = 0;
  float x, y, z, ww, wx, wy, wz;
public:
  void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
  float print_datax(){
    return x;
  }

  float print_datay(){
    return y;
  }

 float print_dataz(){
    return z;
  }

  float print_dataww(){
    return ww; 
  }

  float print_datawx(){
    return wx;
  }

  float print_datawy(){
    return wy;
  }

  float print_datawz(){
    return wz;
  }
};

//  id可以改成其他的
void ARPose::callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    int id = 2;
    if (!msg->markers.empty()) {
    for(int i=0;i<msg->markers.size();i++){
             if(msg->markers[i].id == id){
 	    	id = i;
             }
    }
    }
   // int id = 2;
    //  位置
    x = msg->markers[id].pose.pose.position.x;
    y = msg->markers[id].pose.pose.position.y;
    z = msg->markers[id].pose.pose.position.z;

    //  四元数
    ww = msg->markers[id].pose.pose.orientation.w;
    wx = msg->markers[id].pose.pose.orientation.x;
    wy = msg->markers[id].pose.pose.orientation.y;
    wz = msg->markers[id].pose.pose.orientation.z;
    //x1 = msg->data[0];
    //print_datax();
    //  位置
    print_datax();
    print_datay();
    print_dataz();
    //  四元数
    print_dataww();
    print_datawx();
    print_datawy();
    print_datawz();
    ++count;
   //std::cout<<"ID: "<< x <<std::endl;
}

// 四元数转旋转矩阵
Eigen::Matrix3d Quaternion2RotationMatrix(const double w,const double x,const double y,const double z)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;  
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();   
    return R;  
}

bool pubsh_target(){
    if(pubsh_time<2){

	    cart_pos current_pose, end_pose;
	    if(pubsh_time > 0){

            //  实时获取机器人当前关节角
		    robot_state = robot.receiveRobotState(); 
            //  获取当前位置的其次变换矩阵   	
		    current_pose.pos = robot_state.toolTobase_pos_m;
            //  获取当前位置的其次变换矩阵 
	        end_pose.pos = robot_state.toolTobase_pos_m;
	        end_pose.pos[3] -= pubsh_time * 0.109;
		    MOVEL(0.2, current_pose, end_pose, robot);
		    sleep(1.0);
    	}
        //  实时获取机器人当前关节角
	    robot_state = robot.receiveRobotState();    	
	    current_pose.pos = robot_state.toolTobase_pos_m;
        end_pose.pos = robot_state.toolTobase_pos_m;
	    end_pose.pos[11] -= (0.512180627 - 0.447649264);
        MOVEL(0.2, current_pose, end_pose, robot);
        sleep(1.0);
	
        //  实时获取机器人当前关节角
	    robot_state = robot.receiveRobotState();    	
	    current_pose.pos = robot_state.toolTobase_pos_m;
        end_pose.pos = robot_state.toolTobase_pos_m;
	    end_pose.pos[11] += (0.512180627 - 0.447649264);
        MOVEL(0.2, current_pose, end_pose, robot);
        sleep(1.0);
    }
}

// 机械臂移动到起始位姿
bool robot_move_to_start(){
    std::array<double,7> q_init;
    // std::array<double,7> q_drag = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, robot_start_pose, robot);
    return true;
}

//机械臂移动到识别位姿
bool robot_move_to_identify_ar_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, identify_ar_pose, robot);
    roation_flag = true;
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


// 机械臂移动到抓取位姿
bool robot_move_to_grab_pose(){
    std::array<double,7> q_init;
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2, q_init, identify_ar_pose, robot);
    sleep(1.0);
    return true;
}

// // 机械臂移动到pubsh位姿
// bool robot_move_to_pubsh_pose(){
//     std::array<double,7> q_init;
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState().q;
//     MOVEJ(0.2, q_init, pubsh_pose, robot);
//     sleep(1.0);
//     return true;
// }


// 机械臂移动到准备位姿
bool robot_move_to_ready_pose(){
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,0,0,PI/3,0,PI/2,0}};
    //  实时获取机器人当前关节角
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init, q_drag, robot);
}

//  吸盘动作
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


// //机械臂末端转动
bool robot_roation_move(double roation_z){
    Eigen::Affine3d initial_transform;
    Eigen::Affine3d rot_change;
    Eigen::Affine3d cur_transform;

    std::array<double, 16> init_position;
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose,end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos = robot_state.toolTobase_pos_m;
    init_position = robot_state.toolTobase_pos_m;
    initial_transform = Eigen::Matrix4d::Map(init_position.data()).transpose();
    
    //  旋转顺序ZYX
    rot_change.linear() << Eigen::AngleAxisd(roation_z,Eigen::Vector3d::UnitZ()).toRotationMatrix()*
    Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY()).toRotationMatrix()*
    Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX()).toRotationMatrix();

    cur_transform.linear()<<initial_transform.linear()*rot_change.linear();
    cur_transform.translation() = initial_transform.translation();
    std::array<double, 16> new_pose;
    Eigen::Map<Eigen::Matrix4d>(&new_pose[0], 4, 4) = cur_transform.matrix().transpose();

    end_pose.pos = new_pose;
    std::cout<<current_pose.pos<<std::endl;
    std::cout<<"---------------"<<std::endl;
    std::cout<<end_pose.pos<<std::endl;
    //end_pose.pos[3] += 0.02;
    MOVEL(0.2, current_pose, end_pose, robot);

    sleep(1.0);
}

//机械臂平移运动
bool robot_pan_move_xy(float& x, float& y){
    /*                   转动                                   */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    cout<<end_pose.pos<<endl;

    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x));
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((y));

    cout<<end_pose.pos<<endl;
    MOVEL(0.2, current_pose, end_pose, robot);
    sleep(1.0);
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
    test_DO(1);

    robot_state = robot.receiveRobotState();
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;
    end_pose.pos[11] += 0.038636231;
    MOVEL(0.1, current_pose, end_pose, robot);
    sleep(2.0);

    return true;

}
//机械臂平移运动
bool robot_pan_move_xyz(float& x, float& y, float& z){
    /*                   转动                          -0.08)         */
    //robot_state = robot.receiveRobotState();
    //  实时获取机器人当前关节角
    robot_state = robot.receiveRobotState();
    cart_pos current_pose, end_pose;
    current_pose.pos = robot_state.toolTobase_pos_m;
    end_pose.pos  = robot_state.toolTobase_pos_m;

    // //  x=x',x'为相机的x方向的坐标
    // end_pose.pos[3] += ((x) - 0.014); //0.022
    // //  y方向,y = -y',y'为相机y方向的坐标
    // end_pose.pos[7] += ((y)) -0.020;  // 0.035
    // // z方向
    // end_pose.pos[11] -= (z - 0.30);



    //  x=x',x'为相机的x方向的坐标
    end_pose.pos[3] += ((x) - 0.022); //0.022
    //  y方向,y = -y',y'为相机y方向的坐标
    end_pose.pos[7] += ((y) -0.035);  // 0.035
    // z方向
    end_pose.pos[11]  -= (z - 0.206);

    MOVEL(0.2, current_pose, end_pose, robot);
    sleep(1.0);
}


int main(int argc, char *argv[]) {
    //  初始化参数
    float sx, sy, sz, ww, wx, wy, wz;
    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle n;
    ros::ServiceClient client;
    bool roation_flag = true;

    // 监听ar码,如果识别到AR码就进行抓取动作
    //ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, ar_marker_cb);

    //robot_move_to_grab_pose();

    //  控制机械臂到识别位姿
    //robot_move_to_pubsh_pose();
   
    robot.automaticErrorRecovery();
    sleep(2);
    robot_move_to_identify_ar_pose();
   // pubsh_target();
    //sleep(10.0);

    if (roation_flag){

        // 机械臂转动
        // 获取AR码的位置和四元数
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
            ros::Rate loop_rate(10);
            sleep(2);

            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
            std::cout << "After spin1 : \n";
            sx = arpose.print_datax();
            sy = arpose.print_datay();
            sz = arpose.print_dataz();
            ww = arpose.print_dataww();
            wx = arpose.print_datawx();
            wy = arpose.print_datawy();
            wz = arpose.print_datawz();

            std::cout<<"x: "<< sx <<std::endl;
            std::cout<<"y: "<< sy <<std::endl;
            std::cout<<"z: "<< sz <<std::endl;
            std::cout<<"ww: "<< ww <<std::endl;
            std::cout<<"wx: "<< wx <<std::endl;
            std::cout<<"wy: "<< wy <<std::endl;
            std::cout<<"wz: "<< wz <<std::endl;
            ready_roation = true;

            if(ready_roation){
                identify_time = 1;
            }

            Eigen::Quaterniond quaternion(ww,wx,wy,wz);
            //  四元数求欧拉角
            Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);

            //  将弧度(rad)转换成角度(°)
	        robot_angle = (eulerAngle(0) * 180) / PI;
	        if(robot_angle > 90.0){
	            robot_angle = 180.0 - robot_angle;	
	        }
            std::cout<<"AR码角度： "<< robot_angle <<std::endl;

            if(ready_roation && identify_time!=0){
                if(robot_angle > 90){
                    //  机械臂末端旋转
                    robot_roation_move(-PI/2 + eulerAngle(0));
                    ready_roation = false;
                    identify_time = 0;
                    move_again = true;
                    // std::cout<<"AR码角度： "<<(PI - eulerAngle(0))*57.3<<std::endl;
                }
                else{
                    //  机械臂末端旋转
                    robot_roation_move(eulerAngle(0) - PI/2);
                    // std::cout<<"AR码角度： "<<(eulerAngle(0)-2*PI)*57.3<<std::endl;
                    ready_roation = false;
                    identify_time = 0;
                    move_again = true;
                }

            }

        } while (0);

        // // 获取AR码的位置和四元数
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
            ros::Rate loop_rate(10);
            sleep(2);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }

            std::cout << "After spin2 : \n";
            sx = arpose.print_datax();
            sy = arpose.print_datay();
            sz = arpose.print_dataz();
            ww = arpose.print_dataww();
            wx = arpose.print_datawx();
            wy = arpose.print_datawy();
            wz = arpose.print_datawz();

            std::cout<<"x1: "<< sx <<std::endl;
            std::cout<<"y1: "<< sy <<std::endl;
            std::cout<<"z1: "<< sz <<std::endl;
            std::cout<<"ww1: "<< ww <<std::endl;
            std::cout<<"wx1: "<< wx <<std::endl;
            std::cout<<"wy1: "<< wy <<std::endl;
            std::cout<<"wz1: "<< wz <<std::endl;


            //  机械臂平移
            robot_pan_move_xy(sx, sy);

        } while (0);
    

        // // 获取AR码的位置和四元数
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
            ros::Rate loop_rate(10);
            sleep(2);
            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }

            std::cout << "After spin3 : \n";
            sx = arpose.print_datax();
            sy = arpose.print_datay();
            sz = arpose.print_dataz();
            ww = arpose.print_dataww();
            wx = arpose.print_datawx();
            wy = arpose.print_datawy();
            wz = arpose.print_datawz();

            std::cout<<"x1: "<< sx <<std::endl;
            std::cout<<"y1: "<< sy <<std::endl;
            std::cout<<"z1: "<< sz <<std::endl;
            std::cout<<"ww1: "<< ww <<std::endl;
            std::cout<<"wx1: "<< wx <<std::endl;
            std::cout<<"wy1: "<< wy <<std::endl;
            std::cout<<"wz1: "<< wz <<std::endl;


            if(sy > 0.0616){
                sy = -(sy - 0.0616);
                std::cout << sy <<std::endl;
            }
            else{
                sy = (0.0616 - sy);
            }

            if(sx > 0.0035){
                sx = -(sx - 0.0035);
            }
            else{
                sx = (0.0035 - sx);
            }

            //  机械臂平移
            robot_pan_move_xy(sx, sy);

        } while (0);


        //     // 获取AR码的位置和四元数
        // while(sx == -0.0123) {
        //     ARPose arpose;
        //     ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
        //     ros::Rate loop_rate(10);
        //     sleep(2);

        //     while(ros::ok() and arpose.count <=0){
        //         ros::spin();
        //         loop_rate.sleep();
        //     }
        
        //     std::cout << "After spin4 : \n";
        //     sx = arpose.print_datax();
        //     sy = arpose.print_datay();
        //     sz = arpose.print_dataz();
        //     ww = arpose.print_dataww();
        //     wx = arpose.print_datawx();
        //     wy = arpose.print_datawy();
        //     wz = arpose.print_datawz();

        //     std::cout<<"x1: "<< sx <<std::endl;
        //     std::cout<<"y1: "<< sy <<std::endl;
        //     std::cout<<"z1: "<< sz <<std::endl;
        //     std::cout<<"ww1: "<< ww <<std::endl;
        //     std::cout<<"wx1: "<< wx <<std::endl;
        //     std::cout<<"wy1: "<< wy <<std::endl;
        //     std::cout<<"wz1: "<< wz <<std::endl;
        //     //  机械臂平移
        //     //robot_pan_move_xyz(sx, sy, sz);
        //     ros::spin();
        // }
    

            // 获取AR码的位置和四元数
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
            ros::Rate loop_rate(10);
            sleep(2);

            while(ros::ok() and arpose.count <=0){
                ros::spinOnce();
                loop_rate.sleep();
            }
        
            std::cout << "After spin4 : \n";
            sx = arpose.print_datax();
            sy = arpose.print_datay();
            sz = arpose.print_dataz();
            ww = arpose.print_dataww();
            wx = arpose.print_datawx();
            wy = arpose.print_datawy();
            wz = arpose.print_datawz();

            std::cout<<"x1: "<< sx <<std::endl;
            std::cout<<"y1: "<< sy <<std::endl;
            std::cout<<"z1: "<< sz <<std::endl;
            std::cout<<"ww1: "<< ww <<std::endl;
            std::cout<<"wx1: "<< wx <<std::endl;
            std::cout<<"wy1: "<< wy <<std::endl;
            std::cout<<"wz1: "<< wz <<std::endl;
            //  机械臂平移
            robot_pan_move_xyz(sx, sy, sz);
            test_DO(0);
            //test_DO(1);
            sleep(2);
            //test_DO(1);
        } while (0);
  
    }




    //  机械臂到运动识别位姿
    robot_move_to_identify_ar_pose();
    robot_move_to_pubsh_cen_pose();
    robot_move_to_pubsh_pose();

    robot_push_action(); 

    // 放置
    //robot_move_to_pubsh_pose();
    //sleep(1.0);
    //test_DO(1);
    //robot_push_action();
    ros::spin();
    return 0;

}
