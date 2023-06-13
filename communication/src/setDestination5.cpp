#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
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
};

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
  ros::Publisher waypoint_pub = nh.advertise<geometry_msgs::PoseStamped>((drone+"/mavros/setpoint_position/local"), 10);
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
  wp.push_back(coordinate(10, 0, 2, -90));
  wp.push_back(coordinate(10, 10, 2, -180));
  wp.push_back(coordinate(0, 10, 2, -270));
  wp.push_back(coordinate(0, 0, 2, 0));

  for (int k = 0; k < wp.size(); k++)
  {
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

    while (ros::ok() && distance > POSITION_TOLERANCE)
    {
      ROS_INFO("x:%lf y:%lf z:%lf distance:%lf", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, distance);
      distance = (get_distance(current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));
      ros::spinOnce();
    }
    ROS_WARN("Setpoint (%lf, %lf, %lf) Reached!", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);

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