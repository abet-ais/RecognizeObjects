#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <vector>
#include <time.h>
#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <stdexcept>
#include <std_msgs/Int16.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv){
  gazebo_msgs::GetModelState template_pose_from_world;
  gazebo_msgs::GetModelState camera_pose_from_world;

  ros::init(argc, argv, "send_camera_to_template_tf");
  ros::NodeHandle n;
  ros::ServiceClient get_model_state = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

  while(ros::ok()){
    template_pose_from_world.request.model_name = "linemod_template";
    template_pose_from_world.request.relative_entity_name = "world";
    get_model_state.call(template_pose_from_world);

    geometry_msgs::Point linemod_template_trans = template_pose_from_world.response.pose.position;
    geometry_msgs::Quaternion linemod_template_quat = template_pose_from_world.response.pose.orientation;

    geometry_msgs::Point camera_trans = camera_pose_from_world.response.pose.position;
    geometry_msgs::Quaternion camera_quat = camera_pose_from_world.response.pose.orientation;

    camera_pose_from_world.request.model_name = "abe_kinect";
    camera_pose_from_world.request.relative_entity_name = "world";
    get_model_state.call(camera_pose_from_world);
    
    for(int i=0; i<10; i++){
      static tf::TransformBroadcaster broadcaster;
      tf::Transform world_to_camera_tf;
      world_to_camera_tf.setOrigin(tf::Vector3(camera_trans.x, camera_trans.y, camera_trans.z));
      tf::Quaternion kinect_quat_tf(camera_quat.x, camera_quat.y, camera_quat.z, camera_quat.w);
      world_to_camera_tf.setRotation(kinect_quat_tf);
      broadcaster.sendTransform(tf::StampedTransform(world_to_camera_tf, ros::Time::now(), "world", "linemod_camera"));

      tf::Transform world_to_template_tf;
      world_to_template_tf.setOrigin(tf::Vector3(linemod_template_trans.x, linemod_template_trans.y, linemod_template_trans.z));
      tf::Quaternion template_quat_tf(linemod_template_quat.x, linemod_template_quat.y, linemod_template_quat.z, linemod_template_quat.w);
      world_to_template_tf.setRotation(template_quat_tf);
      broadcaster.sendTransform(tf::StampedTransform(world_to_template_tf, ros::Time::now(), "world", "linemod_template"));
      ros::Duration(0.001).sleep();
    }
  }
  return 0;
}
