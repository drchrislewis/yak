#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <std_srvs/EmptyRequest.h>
#include <std_srvs/EmptyResponse.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "nbv_planner/GetNBV.h"

class Explorer {
public:
  Explorer(ros::NodeHandle &nh);
  bool MoveToNBVs(moveit::planning_interface::MoveGroupInterface &move_group);

private:
  ros::ServiceClient nbv_client_;
  tf::TransformListener tfListener_;
  ros::NodeHandle node_handle_;


};
