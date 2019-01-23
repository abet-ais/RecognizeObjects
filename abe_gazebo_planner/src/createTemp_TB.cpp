//arg[1] カメラとベースの距離を与えてください(m)1.2くらい

#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <time.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <stdexcept>
#include <std_msgs/Int16.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include "abe_gazebo_planner/createTemp_switch.h"


#define base_heitht 1.0
//#define fx 3378.269
#define fx 3373.651
#define fy 3373.651
#define cx 1920
#define cy 1080


geometry_msgs::Pose start_pose;
geometry_msgs::Twist start_twist;
geometry_msgs::Pose kinect_pose;
geometry_msgs::Twist kinect_twist;
geometry_msgs::Vector3 gazebo_pose;

gazebo_msgs::ModelState modelstate;
gazebo_msgs::SetModelState setmodelstate;
gazebo_msgs::ModelState kinect_modelstate;
gazebo_msgs::SetModelState kinect_setmodelstate;
gazebo_msgs::GetModelState template_pose_from_world;
gazebo_msgs::GetModelState camera_pose_from_world;

ros::ServiceClient *client_ptr;
ros::ServiceClient *get_model_state_ptr;
std::string object  = "linemod_template";
float dist_step = 0;
float Distance_CameraParts;



std::vector<std::string> split(std::string& input, char delimiter){
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}



bool updatePoseCb(abe_gazebo_planner::createTemp_switch::Request &req,
		  abe_gazebo_planner::createTemp_switch::Response &res){

  ros::ServiceClient client = (ros::ServiceClient)*client_ptr;
  ros::ServiceClient get_model_state = (ros::ServiceClient)*get_model_state_ptr;

  std::cout << "---------------------------------" << std::endl << std::endl;;

  /* WRS2018 taskboard task */
  static std::vector<double> vec(6,0.0);
  double x, y, z, roll, pitch, yaw;
  const double movement_max_range = 0.2; //配置マットがカメラの目の前だと仮定した時，マットの半分[m]
  const double movement_min_range = -0.2;
  const double resolution = 0.04;          //x,y方向の最小移動文化位置[m]
  static double now_x = movement_max_range;
  static double now_y = movement_max_range;
  
  if( now_x >= movement_min_range ){
    if( now_y >= movement_min_range){
      vec[0] = now_x;
      vec[1] = now_y;
      now_y = now_y - resolution;
    }else{
      now_y = movement_max_range;
      vec[1] = now_y;
      now_x = now_x - resolution;
      vec[0] = now_x;
    }
  }

  vec[3] = 0.0;
  vec[4] = 0.0;
  vec[5] = 0.0;
  
  x = vec[0];
  y = vec[1];
  z = base_heitht;
  pitch = vec[3];
  yaw   = vec[4];
  roll  = vec[5]; 

  /* from[m] to[pixel] */
  int x_pix = (int)(cx-(y*fx)/Distance_CameraParts);
  int y_pix = (int)(cy-(x*fy)/Distance_CameraParts);
  std::cout <<"x_pix:" << x_pix << "  y_pix:" << y_pix << std::endl;

  /*---------------------------------*/

  float camera_position_z = z + Distance_CameraParts;
  std::cout << "camera_distance: " << camera_position_z << std::endl;

  geometry_msgs::Quaternion quaternion;
  tf::Quaternion quat;
  
  //  quat.setEulerZYX(yaw, pitch, -M_PI*0.5);
  quat.setEulerZYX(yaw,pitch,roll);

  //  tf::Quaternion offset_quat = tf::createQuaternionFromRPY(M_PI/2, 0, 0);
  tf::Quaternion offset_quat = tf::createQuaternionFromRPY(0, 0, 0);
  
  quat *= offset_quat;
  quaternionTFToMsg(quat, quaternion);
  //  std::cout<<quaternion.x<<" "<<quaternion.y<<" "<<quaternion.z<<" "<<quaternion.w<<std::endl;

  start_pose.position.x = x;
  start_pose.position.y = y;
  start_pose.position.z = z;
  start_pose.orientation.x = quaternion.x;
  start_pose.orientation.y = quaternion.y;
  start_pose.orientation.z = quaternion.z;
  start_pose.orientation.w = quaternion.w;
  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;
  modelstate.model_name = (std::string)object;
  modelstate.reference_frame = (std::string)"world";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);

  // in-plane rot
  tf::Quaternion inplane_quat = tf::createQuaternionFromRPY(0, 0, 0); 
  tf::Quaternion kinect_offset_quat = tf::createQuaternionFromRPY(0, M_PI/2, 0);
  inplane_quat *= kinect_offset_quat;
  geometry_msgs::Quaternion kinect_quat;
  quaternionTFToMsg(inplane_quat, kinect_quat);

  kinect_pose.position.x = 0;
  kinect_pose.position.y = 0;
  kinect_pose.position.z = camera_position_z;
  kinect_pose.orientation.x = kinect_quat.x;
  kinect_pose.orientation.y = kinect_quat.y;
  kinect_pose.orientation.z = kinect_quat.z;
  kinect_pose.orientation.w = kinect_quat.w;
  kinect_twist.linear.x = 0.0;
  kinect_twist.linear.y = 0.0;
  kinect_twist.linear.z = 0.0;
  kinect_twist.angular.x = 0.0;
  kinect_twist.angular.y = 0.0;
  kinect_twist.angular.z = 0.0;
  kinect_modelstate.model_name = (std::string) "abe_kinect";
  kinect_modelstate.reference_frame = (std::string) "world";
  kinect_modelstate.pose = kinect_pose;
  kinect_modelstate.twist = kinect_twist;
  kinect_setmodelstate.request.model_state = kinect_modelstate;
  client.call(kinect_setmodelstate);

  template_pose_from_world.request.model_name = "linemod_template";
  template_pose_from_world.request.relative_entity_name = "world";
  get_model_state.call(template_pose_from_world);

  for(int i=0; i<2; i++){
    static tf::TransformBroadcaster broadcaster;
    tf::Transform world_to_camera_tf;
    tf::Quaternion kinect_quat_tf(kinect_quat.x, kinect_quat.y, kinect_quat.z, kinect_quat.w);
    world_to_camera_tf.setOrigin(tf::Vector3(0, 0, camera_position_z));
    world_to_camera_tf.setRotation(kinect_quat_tf);
    broadcaster.sendTransform(tf::StampedTransform(world_to_camera_tf, ros::Time::now(), "world", "linemod_camera"));

    tf::Transform world_to_template_tf;
    world_to_template_tf.setOrigin(tf::Vector3(0, 0, base_heitht));
    world_to_template_tf.setRotation(quat);
    broadcaster.sendTransform(tf::StampedTransform(world_to_template_tf, ros::Time::now(), "world", "linemod_template"));

    ros::Duration(0.001).sleep();
  }

  
  res.x_pixel = x_pix;
  res.y_pixel = y_pix;
  res.check = req.movement;
  int check = res.check;
  std::cout <<"check:"<<check<<" x_p:"<<x_pix<<" y_p:"<<y_pix << std::endl;
  return true;
}



int main(int argc, char **argv){
  ros::init(argc, argv, "turtlebot_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::ServiceClient get_model_state = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  client_ptr = &client;
  get_model_state_ptr = &get_model_state;

  ros::ServiceServer switch_server = n.advertiseService("sw_pose",updatePoseCb);

  Distance_CameraParts = std::stof(argv[1]);

  ros::spin();
  return 0;
}
