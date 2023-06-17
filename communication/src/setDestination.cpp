#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <unistd.h>
#include <time.h>
#include <ros/duration.h>
#include <cmath>
#include <vector>
#include <tuple>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

std_msgs::Bool drone1;
std_msgs::Bool drone2;
std_msgs::Bool drone3;
std_msgs::Bool drone4;
std_msgs::Bool drone5;

int32_t drone1_cmd;
int32_t drone2_cmd;
int32_t drone3_cmd;
int32_t drone4_cmd;
int32_t drone5_cmd;

// std::vector<int32_t> v;
int32_t v[5];



const float POSITION_TOLERANCE = 0.1;
const float ON_TARGET_THRESHOLD = 20;
const float PI = 3.1428;

class coordinate
{
public:
  double x;
  double y;
  double z;
  double psi;

  coordinate(double x, double y, double z, double psi)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->psi = psi;
  }

  coordinate operator-(coordinate &c){
    return coordinate(this->x - c.x,this->y - c.y,this->z,this->psi);
  }

};

// int32_t min_vecs(std::vector<int32_t> vec){

//   int32_t value=-1;
//   for(int i=0;i<vec.size();i++){
//     if()
//   }



// }

int32_t min_woah(int* v){

int32_t min=10;
for(int i=0;i<5;i++){

  if(v[i]<min){
    min=v[i];
  }

}
return min;

}


void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
  ROS_INFO("CONNECTED!");
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr pose)
{
  current_pose = *pose;
}

float get_distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2) + pow(z1 - z2, 2));
}

void drone1_cb(const std_msgs::Bool::ConstPtr &msg)
{
  ROS_INFO("Drone1 has reached = %d",msg->data);
  drone1.data=msg->data;
}


void drone2_cb(const std_msgs::Bool::ConstPtr &msg)
{
  ROS_INFO("drone2 has reached = %d",msg->data);
  drone2.data=msg->data;
}



void drone3_cb(const std_msgs::Bool::ConstPtr &msg)
{
  ROS_INFO("Drone3 has reached = %d",msg->data);
  drone3.data=msg->data;
}



void drone4_cb(const std_msgs::Bool::ConstPtr &msg)
{
  ROS_INFO("Drone4 has reached = %d",msg->data);
  drone4.data=msg->data;
}



void drone5_cb(const std_msgs::Bool::ConstPtr &msg)
{
  ROS_INFO("Drone5 has reached = %d",msg->data);
  drone5.data=msg->data;
}





void drone1_cmd_cb(const std_msgs::Int32::ConstPtr &msg)
{
  // ROS_INFO("Drone1 has reached = %d",msg->data);
  drone1_cmd=msg->data;
  v[0]=msg->data;
}

void drone2_cmd_cb(const std_msgs::Int32::ConstPtr &msg)
{
  // ROS_INFO("Drone1 has reached = %d",msg->data);
  drone2_cmd=msg->data;
  v[1]=msg->data;

}

void drone3_cmd_cb(const std_msgs::Int32::ConstPtr &msg)
{
  // ROS_INFO("Drone1 has reached = %d",msg->data);
  drone3_cmd=msg->data;
  v[2]=msg->data;

}

void drone4_cmd_cb(const std_msgs::Int32::ConstPtr &msg)
{
  // ROS_INFO("Drone1 has reached = %d",msg->data);
  drone4_cmd=msg->data;
  v[3]=msg->data;
}

void drone5_cmd_cb(const std_msgs::Int32::ConstPtr &msg)
{
  // ROS_INFO("Drone1 has reached = %d",msg->data);
  drone5_cmd=msg->data;
  v[4]=msg->data;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;
  std::string drone;

  nh.getParam("square1/namespace",drone);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(10.0);
  
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>((drone+"/mavros/state"), 10, state_cb);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>((drone+"/mavros/local_position/pose"), 10, pose_cb);
  
  ros::Subscriber dead_drone1 = nh.subscribe<std_msgs::Bool>("/drone1/mavros/reached",10,drone1_cb);
  ros::Subscriber dead_drone2 = nh.subscribe<std_msgs::Bool>("/drone2/mavros/reached",10,drone2_cb);
  ros::Subscriber dead_drone3 = nh.subscribe<std_msgs::Bool>("/drone3/mavros/reached",10,drone3_cb);
  ros::Subscriber dead_drone4 = nh.subscribe<std_msgs::Bool>("/drone4/mavros/reached",10,drone4_cb);
  ros::Subscriber dead_drone5 = nh.subscribe<std_msgs::Bool>("/drone5/mavros/reached",10,drone5_cb);

  ros::Subscriber cmd_drone1 = nh.subscribe<std_msgs::Int32>("/drone1/mavros/cmdno",10,drone1_cmd_cb);
  ros::Subscriber cmd_drone2 = nh.subscribe<std_msgs::Int32>("/drone2/mavros/cmdno",10,drone2_cmd_cb);
  ros::Subscriber cmd_drone3 = nh.subscribe<std_msgs::Int32>("/drone3/mavros/cmdno",10,drone3_cmd_cb);
  ros::Subscriber cmd_drone4 = nh.subscribe<std_msgs::Int32>("/drone4/mavros/cmdno",10,drone4_cmd_cb);
  ros::Subscriber cmd_drone5 = nh.subscribe<std_msgs::Int32>("/drone5/mavros/cmdno",10,drone5_cmd_cb);


  
  ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>((drone+"/mavros/setpoint_position/local"), 10);
  ros::Publisher deadlock = nh.advertise<std_msgs::Bool>(drone+"/mavros/reached",10);
  ros::Publisher command_no = nh.advertise<std_msgs::Int32>(drone+"/mavros/cmdno",10);

  // ros::Publisher deadlock = nh.advertise<
  // wait for FCU connection
  
  
  while (ros::ok() && !current_state.connected)
  {

    ROS_INFO("%s,connecting...",drone.c_str());
    ros::spinOnce();
      rate.sleep();
  }

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(drone+"/mavros/set_mode");

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "GUIDED";
  if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    ROS_INFO("OFFBOARD enabled");
  else  
  {
    ROS_WARN("unable to switch to offboard");
    return -1;
  }

  sleep(2);

  // arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>(drone+"/mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
    ROS_WARN("ARM sent %d", srv_arm_i.response.success);
  else
  {
    ROS_ERROR("Failed arming/disarming");
    // return -1;
  }

  sleep(2);

  ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>(drone+"/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 2;
  srv_takeoff.request.min_pitch = 0;
  srv_takeoff.request.yaw = 0;
  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_WARN("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }
  ROS_INFO("Takeoff Successful!");

  if (takeoff_client.call(srv_takeoff) && srv_takeoff.response.success)
    ROS_WARN("takeoff sent %d", srv_takeoff.response.success);
  else
  {
    ROS_ERROR("Failed Takeoff");
    return -1;
  }
  ROS_INFO("Takeoff Successful!");



  sleep(5);

  // list of waypoints
  ROS_INFO("Mission Initiated");

  std::vector<coordinate> wp;

  coordinate pose1(0,0,2,0);
  coordinate pose2(0,10,2,0);
  coordinate pose3(0,-10,2,0);
  coordinate pose4(-10,0,2,0);
  coordinate pose5(10,0,2,0);



if(drone=="/drone1"){
  
  wp.push_back(coordinate(10, 0, 2, -90));
  wp.push_back(coordinate(10, 10, 2, -180));
  wp.push_back(coordinate(0, 10, 2, -270));
  wp.push_back(coordinate(0, 0, 2, 0));
  
  wp.push_back(coordinate(0, 0, 2, 0));

  wp.push_back(coordinate(0, 10, 2, 0));

  wp.push_back(coordinate(0, 0, 2, 0));

  wp.push_back(coordinate(0, 0, 2, 0));

  wp.push_back(coordinate(0, 0, 2, 0));

  wp.push_back(coordinate(0, 10, 2, 0));

  wp.push_back(coordinate(0, 0, 2, 0));


}else if(drone=="/drone2"){

  wp.push_back(coordinate(10, 0, 2, -90));
  wp.push_back(coordinate(10, 10, 2, -180));
  wp.push_back(coordinate(0, 10, 2, -270));
  wp.push_back(coordinate(0, 0, 2, 0));
  
  wp.push_back(coordinate(-20, 0, 2, 0)-pose2);
  
  wp.push_back(coordinate(-10, -10, 2, 0)-pose2);

  wp.push_back(coordinate(-20, 0, 2, 0)-pose2);

  wp.push_back(coordinate(-10, -10, 2, 0)-pose2);

  wp.push_back(coordinate(-20, 0, 2, 0)-pose2);

  wp.push_back(coordinate(-9.51, 3.09, 2, 0)-pose2);

  wp.push_back(coordinate(-20, 0, 2, 0)-pose2);


}else if(drone=="/drone3"){

  wp.push_back(coordinate(10, 0, 2, -90));
  wp.push_back(coordinate(10, 10, 2, -180));
  wp.push_back(coordinate(0, 10, 2, -270));
  wp.push_back(coordinate(0, 0, 2, 0));
  
  wp.push_back(coordinate(20, 0, 2, 0)-pose3);

  wp.push_back(coordinate(10, -10, 2, 0)-pose3);

  wp.push_back(coordinate(20, 0, 2, 0)-pose3);

  wp.push_back(coordinate(10, -10, 2, 0)-pose3);

  wp.push_back(coordinate(20, 0, 2, 0)-pose3);

  wp.push_back(coordinate(9.51, 3.09, 2, 0)-pose3);

  wp.push_back(coordinate(20, 0, 2, 0)-pose3);


}else if(drone=="/drone4"){

  wp.push_back(coordinate(10, 0, 2, -90));
  wp.push_back(coordinate(10, 10, 2, -180));
  wp.push_back(coordinate(0, 10, 2, -270));
  wp.push_back(coordinate(0, 0, 2, 0));
  
  wp.push_back(coordinate(-10, 0, 2, 0)-pose4);

  wp.push_back(coordinate(-5, 0, 2, 0)-pose4);

  wp.push_back(coordinate(-10, 0, 2, 0)-pose4);

  wp.push_back(coordinate(-10, 10, 2, 0)-pose4);

  wp.push_back(coordinate(-10, 0, 2, 0)-pose4);

  wp.push_back(coordinate(-5.88, -8.09, 2, 0)-pose4);

  wp.push_back(coordinate(-10, 0, 2, 0)-pose4);


}else if(drone=="/drone5"){

  wp.push_back(coordinate(10, 0, 2, -90));
  wp.push_back(coordinate(10, 10, 2, -180));
  wp.push_back(coordinate(0, 10, 2, -270));
  wp.push_back(coordinate(0, 0, 2, 0));
  
  wp.push_back(coordinate(10, 0, 2, 0)-pose5);

  wp.push_back(coordinate(5, 0, 2, 0)-pose5);

  wp.push_back(coordinate(10, 0, 2, 0)-pose5);

  wp.push_back(coordinate(10, 10, 2, 0)-pose5);

  wp.push_back(coordinate(10, 0, 2, 0)-pose5);

  wp.push_back(coordinate(5.88, -8.09, 2, 0)-pose5);

  wp.push_back(coordinate(10, 0, 2, 0)-pose5);


}


  for (int k = 0; k < wp.size(); k++)
  {


    std_msgs::Int32 cmd_no;
    cmd_no.data=k+1;
    command_no.publish(cmd_no);
    int32_t val = min_woah(v);

    while(ros::ok() && cmd_no.data != val ){      
      val = min_woah(v);
      command_no.publish(cmd_no);
      ROS_WARN("%s,%d,%d",drone.c_str(),cmd_no.data,val);
      ros::spinOnce();
    }


    

    geometry_msgs::PoseStamped msg;
    geometry_msgs::Point point;
    double yaw = wp[k].psi * (PI / 100);
    double pitch = 0.0;
    double roll = 0.0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    double qw = cy * cr * cp + sy * sr * sp;
    double qx = cy * sr * cp - sy * cr * sp;
    double qy = cy * cr * sp + sy * sr * cp;
    double qz = sy * cr * cp - cy * sr * sp;

    point.x = wp[k].x;
    point.y = wp[k].y;
    point.z = wp[k].z;
    msg.pose.position = point;

    waypoint_pub.publish(msg);

    double distance = (get_distance(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));

    std_msgs::Bool dead_bool;
    
    while (ros::ok() && distance > POSITION_TOLERANCE)
    {
      dead_bool.data = false;
      deadlock.publish(dead_bool);
      ROS_INFO("x:%lf y:%lf z:%lf distance:%lf", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, distance);
      distance = (get_distance(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
      ros::spinOnce();
    }
    
    ROS_WARN(" Setpoint (%lf, %lf, %lf) Reached!", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

    
    val=min_woah(v);

    while(ros::ok() && !(drone1.data&drone2.data&drone3.data&drone4.data&drone5.data)&& cmd_no.data>val ){      
      val = min_woah(v);
      ROS_WARN("%d bottom",val);
      dead_bool.data = true;
      deadlock.publish(dead_bool);
      ros::spinOnce();
    }



    sleep(5);
  }

  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>(drone+"/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  srv_land.request.min_pitch = 0;
  srv_land.request.yaw = 0;
  srv_land.request.altitude = 0;
  srv_land.request.longitude = 0;
  srv_land.request.latitude = 0;
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_WARN("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Failed Land");
    return -1;
  }
  sleep(10);
  ROS_INFO("Land Successful!");
  ROS_INFO("MISSION SUCCESSFUL");

  return 0;
}




// std::vector<double> pose_estimate(cv::Mat &frame, cv::Mat &camMat, cv::Mat &dist_coeff)
// {
//     // static tictoc timeCheck("pose");

//     std::vector<double> b = {-1000, -1000, -1000, -1000, -1000, -1000};
//     std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
//     std::vector<int> markerIds;

//     // parameters->

//     // timeCheck.tic();
//     cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
//     // timeCheck.toc_print();

//     if (markerIds.size() != 0)
//     {
//         std::vector<cv::Vec3d> rvec, tvec;
//         cv::aruco::estimatePoseSingleMarkers(markerCorners, MARKER_SIZE, camMat, dist_coeff, rvec, tvec);
//         int i = 0;
//         for (auto &it : markerIds)
//         {
//             if (it == 17) // ID for 1st drone is 0
//             {
//                 b[0] = tvec[i][0];
//                 b[1] = tvec[i][1];
//                 b[2] = DIST_CEILING - tvec[i][2];
//             }
//             else if (it == 10) // ID for 2nd drone is 10
//             {
//                 b[3] = tvec[i][0];
//                 b[4] = tvec[i][1];
//                 b[5] = DIST_CEILING - tvec[i][2];
//             }
//             i++;
//         }
//     }
//     return b;
// }