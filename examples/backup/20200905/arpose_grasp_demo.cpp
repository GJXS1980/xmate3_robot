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

using namespace xmate;
using namespace std;
using namespace Eigen;

using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
using CartesianControl = std::function<CartesianPose(RCI::robot::RobotState robot_state)>;

#include <iostream>
 
using namespace std;

// ar码ID
// int ar_marker_id = 2;   //  ar码id

class ARPose
{
public:
  int count = 0;
  float x, y, z, ww, wx, wy, wz;
public:
  void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
  float print_datax()
  {
  return x;
  }

  float print_datay()
  {

  return y;
  }

 float print_dataz()
  {

  return z;
  }

  float print_dataww()
  {

  return ww; 
  }

  float print_datawx()
  {

  return wx;
  }

  float print_datawy()
  {

  return wy;
  }

  float print_datawz()
  {

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
   std::cout<<"ID: "<< x <<std::endl;
 
}



// // 写死  机械臂连接
// std::string ipaddr = "192.168.1.60";    // 机器人ip
// uint16_t port = 1337;   //  机器人端口
// xmate::Robot robot(ipaddr, port);
// RCI::robot::RobotState robot_state; //  机器人状态

// const double PI = 3.14159;

// // ar码pose
// geometry_msgs::PoseStamped ar_traget_pose;  //  ar码的时间戳

// // ar码ID
// int ar_marker_id = 2;   //  ar码id

// double robot_angle = 0.0;

// std_msgs::String target_msg;

// bool ready_to_grap = false; //  抓取的flag

// int pubsh_time = 0;

// //机械臂起始位姿
// std::array<double, 7> robot_start_pose = {{0,0,PI/2,PI/2,0,PI/2,0}};

// //机械臂识别位姿
// std::array<double, 7> identify_ar_pose = {{0, -PI/18, 0, -PI/2.4, 0,-PI/2, 0}};

// //机械臂抓取位姿
// //std::array<double, 7> grab_pose = {{0, 0, 0, PI/2, 0,PI/2, 0}};
// std::array<double,7> grab_pose = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};

// //fangzhi x -  z+ 531.77845 - 446.70611
// std::array<double,7> pubsh_pose = {{-1.3221816636573167,-0.15919988174970715, -0.04852340758825353, -1.0854939408299562, 0.12496011229946233,-1.7362460417538408, 0.0}};

// /*--------------------------------------------------------*/

// // 四元数转旋转矩阵
// Eigen::Matrix3d Quaternion2RotationMatrix(const double w,const double x,const double y,const double z)  
// {  
//     Eigen::Quaterniond q;  
//     q.x() = x;  
//     q.y() = y;  
//     q.z() = z;  
//     q.w() = w;  
  
//     Eigen::Matrix3d R = q.normalized().toRotationMatrix();   
    
//     return R;  
// }

// bool pubsh_target(){
//     if(pubsh_time<2){
// 	cart_pos current_pose, end_pose;
// 	if(pubsh_time > 0){
//         //  实时获取机器人当前关节角
// 		robot_state = robot.receiveRobotState(); 
//         //  获取当前位置的其次变换矩阵   	
// 		current_pose.pos = robot_state.toolTobase_pos_m;
//         //  获取当前位置的其次变换矩阵 
// 	    end_pose.pos = robot_state.toolTobase_pos_m;
// 	    end_pose.pos[3] -= pubsh_time * 0.109;
// 		MOVEL(0.2, current_pose, end_pose, robot);
// 		sleep(1.0);
//     	}
//     //  实时获取机器人当前关节角
// 	robot_state = robot.receiveRobotState();    	
// 	current_pose.pos = robot_state.toolTobase_pos_m;
//     end_pose.pos = robot_state.toolTobase_pos_m;
// 	end_pose.pos[11] -= (0.512180627 - 0.447649264);
//     MOVEL(0.2, current_pose, end_pose, robot);
//     sleep(1.0);
	
//     //  实时获取机器人当前关节角
// 	robot_state = robot.receiveRobotState();    	
// 	current_pose.pos = robot_state.toolTobase_pos_m;
//     end_pose.pos = robot_state.toolTobase_pos_m;
// 	end_pose.pos[11] += (0.512180627 - 0.447649264);
//     MOVEL(0.2, current_pose, end_pose, robot);
//     sleep(1.0);
//     }
// }

// // 机械臂移动到起始位姿
// bool robot_move_to_start(){
//     std::array<double,7> q_init;
//     // std::array<double,7> q_drag = {{0,-PI/6,0,-PI/3,0,-PI/2,0}};
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState().q;
//     MOVEJ(0.2, q_init, robot_start_pose, robot);
//     return true;
// }

// //机械臂移动到识别位姿
// bool robot_move_to_identify_ar_pose(){
//     std::array<double,7> q_init;
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState().q;
//     MOVEJ(0.2, q_init, identify_ar_pose, robot);
//     return true;
// }

// // 机械臂移动到抓取位姿
// bool robot_move_to_grab_pose(){
//     std::array<double,7> q_init;
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState().q;
//     MOVEJ(0.2, q_init, identify_ar_pose, robot);
//     sleep(1.0);
//     return true;
// }

// // 机械臂移动到pubsh位姿
// bool robot_move_to_pubsh_pose(){
//     std::array<double,7> q_init;
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState().q;
//     MOVEJ(0.2, q_init, pubsh_pose, robot);
//     sleep(1.0);
//     return true;
// }

// // 机械臂移动到准备位姿
// bool robot_move_to_ready_pose(){
//     std::array<double,7> q_init;
//     std::array<double,7> q_drag = {{0,0,0,PI/3,0,PI/2,0}};
//     //  实时获取机器人当前关节角
//     q_init = robot.receiveRobotState(
           // ARPose arpose;).q;
//     MOVEJ(0.2,q_init, q_drag, robot);
// }

// //机械臂平移运动
// bool robot_pan_move(double roation_z){

//     //robot_state = robot.receiveRobotState();
//     //  实时获取机器人当前关节角
//     robot_state = robot.receiveRobotState();
//     cart_pos current_pose, end_pose;
//     current_pose.pos = robot_state.toolTobase_pos_m;
//     end_pose.pos  = robot_state.toolTobase_pos_m;

//     //  x方向,x = -y'
//     end_pose.pos[3] += ((-ar_traget_pose.pose.position.y + 0.038596));//-0.01);//- 0.06) + 0.02; //-0.114021	
    
//     //  y方向,y = x
//     end_pose.pos[7] += ((ar_traget_pose.pose.position.x - 0.114021));//-0.08); //- 0.030);//+ end_y_less;
    
//     cout<<end_pose.pos<<endl;
//     MOVEL(0.2, current_pose, end_pose, robot);
//     sleep(1.0);

//     // z方向
//     //  实时获取机器人当前关节角
//     robot_state = robot.receiveRobotState();
//     current_pose.pos = robot_state.toolTobase_pos_m;
//     end_pose.pos  = robot_state.toolTobase_pos_m;
//     //0.22 -0.15-0.008-0.018
//     // z方向
//     //end_pose.pos[11] -= (ar_traget_pose.pose.position.z - 0.138 - 0.008);
//     //  坐标变换
//     //MOVEL(0.2, current_pose, end_pose, robot);
//     cout << "AR pose: " << end_pose.pos << endl;
//     sleep(2.0);
    
// }


// // 监听ar码回调
// void ar_marker_cb(ar_track_alvar_msgs::AlvarMarkers req) {
//     //ar_marker_id = count*2;
//     if (!req.markers.empty() && ar_marker_id != -1) {
// 	int id = -1;
// 	for(int i=0;i<req.markers.size();i++){
//             if(req.markers[i].id == ar_marker_id){
// 	    	id = i;
//             }
//         }
//         float wx = req.markers[id].pose.pose.orientation.x;
//         float wy = req.markers[id].pose.pose.orientation.y;
//         float wz = req.markers[id].pose.pose.orientation.z;
//         float ww = req.markers[id].pose.pose.orientation.w;

//         float x = req.markers[id].pose.pose.position.x;
//         float y = req.markers[id].pose.pose.position.y;
//         float z = req.markers[id].pose.pose.position.z;
        
//         // std::cout<<"orientation: "<< wx << wy << wz << ww<<std::endl;
//         // std::cout<<"postion: "<< x << y <<z << std::endl;

//         ar_traget_pose.pose.position = req.markers[id].pose.pose.position;
//         ar_traget_pose.pose.orientation = req.markers[id].pose.pose.orientation;
        
//         Eigen::Quaterniond quaternion(ww,wx,wy,wz);
//         Eigen::Vector3d eulerAngle=quaternion.matrix().eulerAngles(2,1,0);
//         // ZYX - RPY
//         //std::cout<<"欧拉角: "<< eulerAngle<<std::endl;
//         //std::cout<<"-----------"<<std::endl;
//         std::cout<<"AR码角度： "<<eulerAngle(0)*57.3<<std::endl;
// 	robot_angle = eulerAngle(0)*57.3;
// 	if(robot_angle>90.0){
// 	   robot_angle = 180.0 - robot_angle;	
// 	}
//         std::cout<<"末端旋转角度："<<robot_angle<<std::endl;

//         //std::cout<<"ID: "<<ar_marker_id<<","<<"ar_traget"<<ar_traget_pose<<std::endl;
        
//         if(ready_to_grap && (x != 0.0 || y != 0.0 || z != 0.0)){
//             robot_pan_move(eulerAngle(0)-PI);
// 	    ready_to_grap = false;
// 	    sleep(2.0);
// 	}
//     }
// }

int main(int argc, char *argv[]) {
    //  初始化参数
    float sx, sy, sz, ww, wx, wy, wz;
    /*ros 初始化*/
    ros::init(argc, argv, "ros_core");
    ros::NodeHandle n;
    ros::ServiceClient client;

    // 监听ar码,如果识别到AR码就进行抓取动作
    //ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, ar_marker_cb);

    //robot_move_to_grab_pose();

    //  控制机械臂到识别位姿
    //robot_move_to_pubsh_pose();
   // pubsh_target();
    //sleep(2.0);
    
        // 获取AR码的位置和四元数
        do {
            ARPose arpose;
            ros::Subscriber ar_pose = n.subscribe("ar_pose_marker", 1000, &ARPose::callback, &arpose);
            ros::Rate loop_rate(10);
            //sleep(2);

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

            std::cout << sx << "\n";
            std::cout << sy << "\n";
            std::cout << sz << "\n";
            std::cout << ww << "\n";
            std::cout << wx << "\n";
            std::cout << wy << "\n";
            std::cout << wz << "\n";
            } while (0);

        // // 获取AR码的位置和四元数
        // do {
        //     ARPose arpose;
        //     ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
        //     ros::Rate loop_rate(10);

        //     while(ros::ok() and arpose.count <=7){
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //     }
        //     std::cout << "After spin2 : \n";
        //     sx = arpose.print_datax();
        //     sy = arpose.print_datay();
        //     sz = arpose.print_dataz();
        //     ww = arpose.print_dataww();
        //     wx = arpose.print_datawx();
        //     wy = arpose.print_datawy();
        //     wz = arpose.print_datawz();

        //     std::cout << sx << "\n";
        //     std::cout << sy << "\n";
        //     std::cout << sz << "\n";
        //     std::cout << ww << "\n";
        //     std::cout << wx << "\n";
        //     std::cout << wy << "\n";
        //     std::cout << wz << "\n";
        //     } while (0);


        // // 获取AR码的位置和四元数
        // do {
        //     ARPose arpose;
        //     ros::Subscriber ar_pose = core.subscribe("ar_pose_marker", 1, &ARPose::callback, &arpose);
        //     ros::Rate loop_rate(10);

        //     while(ros::ok() and arpose.count <=7){
        //         ros::spinOnce();
        //         loop_rate.sleep();
        //     }
        //     std::cout << "After spin3 : \n";
        //     sx = arpose.print_datax();
        //     sy = arpose.print_datay();
        //     sz = arpose.print_dataz();
        //     ww = arpose.print_dataww();
        //     wx = arpose.print_datawx();
        //     wy = arpose.print_datawy();
        //     wz = arpose.print_datawz();

        //     std::cout << sx << "\n";
        //     std::cout << sy << "\n";
        //     std::cout << sz << "\n";
        //     std::cout << ww << "\n";
        //     std::cout << wx << "\n";
        //     std::cout << wy << "\n";
        //     std::cout << wz << "\n";
        //     } while (0);


    //ready_to_grap = true;
    sleep(1);

    ros::spin();
    //return 0;
}
