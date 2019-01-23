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

#define INITIAL_DIST 1.19
#define Z 2
float temp_z = 0.0;

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
std::string coke = "coke_can";
std::string line;
std::ifstream ifs("/home/ais/ais-project/catkin_ws/src/devel/abe/WRS2018/WRCVision/auto_temp/abe_sandbox/data/gazebo_hemi_icosa_div4.csv");
ros::Publisher pose_flag_pub;
std_msgs::Int16 pose_flag;
float dist_step = 0;

std::vector<std::string> split(std::string& input, char delimiter){
  std::istringstream stream(input);
  std::string field;
  std::vector<std::string> result;
  while (getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void updatePoseCb(const std_msgs::Int16 msg){
  ros::ServiceClient client = (ros::ServiceClient)*client_ptr;
  ros::ServiceClient get_model_state = (ros::ServiceClient)*get_model_state_ptr;

  getline(ifs, line);
  std::vector<std::string> strvec = split(line, ' ');

  std::vector<double > vec((int)strvec.size());
  std::cout << "---------------------------------" << std::endl << std::endl;;
  for (int i=0; i<strvec.size();i++){
    try{
      vec[i] = std::stod(strvec[i]);
      std::cout << vec[i] << ", " << i <<  std::endl;
    } catch(std::invalid_argument e) {
      std::cout << "error! check csv file" << std::endl;
    }
  }

  if(pose_flag.data == 1){
    dist_step = dist_step + 0.1;
  }

  double x, y, z, roll, pitch, yaw; 
  //theta: z軸と半径の間の角度
  // x, y, z, phi, theta, yaw
  pitch = vec[3]; // theta
  yaw = vec[4]; // phi
  

  x = 0;
  y = 0;
  z = Z;
  
  float camera_position_z = z + INITIAL_DIST + dist_step;
  std::cout << "camera_distance: " << camera_position_z << std::endl;
  pose_flag.data = msg.data;

  geometry_msgs::Quaternion quaternion;
  // tf::Quaternion quat = tf::createQuaternionFromRPY(0,pitch,yaw);
  tf::Quaternion quat;

  quat.setEulerZYX(yaw, pitch, 0);
  // quat.setEulerZYX(0, vec[1], 0);

  // start after rotate roll for pi/2 *original means gazebo origin coords
  tf::Quaternion offset_quat = tf::createQuaternionFromRPY(M_PI/2, 0, 0);
  // tf::Quaternion offset_quat = tf::createQuaternionFromRPY(0, 0, 0);

  quat *= offset_quat;
  quaternionTFToMsg(quat, quaternion);

  start_pose.position.x = x;
  start_pose.position.y = y;
  start_pose.position.z = z - temp_z;
  start_pose.orientation.x = quaternion.x;
  start_pose.orientation.y = quaternion.y;
  start_pose.orientation.z = quaternion.z;
  // start_pose.orientation.w = quaternion.w;//add abe
  
  std::cout << quaternion.x <<" "<< quaternion.y <<" "<< quaternion.z <<" "<<quaternion.w<< std::endl;

  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;
  modelstate.model_name = (std::string) object;
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;
  //set model state 
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);

  // in-plane rot
  tf::Quaternion inplane_quat = tf::createQuaternionFromRPY(0, 0, vec[5]); 
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

  // camera_pose_from_world.request.model_name = "abe_kinect";
  // camera_pose_from_world.request.relative_entity_name = "world";
  // get_model_state.call(camera_pose_from_world);
    
  for(int i=0; i<10; i++){
    static tf::TransformBroadcaster broadcaster;
    tf::Transform world_to_camera_tf;
    world_to_camera_tf.setOrigin(tf::Vector3(0, 0, camera_position_z));
    tf::Quaternion kinect_quat_tf(kinect_quat.x, kinect_quat.y, kinect_quat.z, kinect_quat.w);
    world_to_camera_tf.setRotation(kinect_quat_tf);
    broadcaster.sendTransform(tf::StampedTransform(world_to_camera_tf, ros::Time::now(), "world", "linemod_camera"));

    tf::Transform world_to_template_tf;
    world_to_template_tf.setOrigin(tf::Vector3(0, 0, 2));
    world_to_template_tf.setRotation(quat);
    broadcaster.sendTransform(tf::StampedTransform(world_to_template_tf, ros::Time::now(), "world", "linemod_template"));
    ros::Duration(0.001).sleep();
  }
  pose_flag_pub.publish(pose_flag);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "turtlebot_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  ros::ServiceClient get_model_state = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  client_ptr = &client;
  get_model_state_ptr = &get_model_state;
  ros::Subscriber pose_sub = n.subscribe("createFrag", 1000, updatePoseCb);
  pose_flag_pub = n.advertise<std_msgs::Int16>("poseFrag",1000);

  temp_z = std::stof(argv[1]);

  ros::spin();
  return 0;
}
