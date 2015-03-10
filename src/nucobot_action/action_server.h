/*
 * action_server.h
 *
 *  Created on: Feb 16, 2015
 *      Author: vsevolod
 */

#include <cmath>
#include <climits>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>

// base class for action server
#include <actionlib/server/simple_action_server.h>

// it is necessary to communicate with move_base
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

// Action description files (check the /action dir)
#include <nucobot_action/AchieveTargetAction.h>

#ifndef ACTION_SERVER_H_
#define ACTION_SERVER_H_

////////////////////////////////////////////////////////////////////////////////
//  ActionServer
////////////////////////////////////////////////////////////////////////////////

class ActionServer
{
private:
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<nucobot_action::AchieveTargetAction>   as_achieve_target;

    // MoveBase ActionClient
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac;

    // Motion variables
    geometry_msgs::Twist buf_cmd;
    geometry_msgs::PoseStamped position;
    geometry_msgs::Pose2D target;
    std::string target_name;
    gazebo_msgs::ModelStates obj_map;
    bool need_clear_map;
    bool is_under_planner_control;

public:
    ActionServer(ros::NodeHandle nh_);
    ~ActionServer(void) {}

    void achieveTargetCB(const nucobot_action::AchieveTargetGoalConstPtr  &goal);

    // Motion functions
    bool send_target_to_move_base();
    bool cancel_move_base_goal();
    bool cancel_move_base_goal(double eps);
    double get_yaw_to_target();
    bool angle_approach();
    bool linear_approach();
    geometry_msgs::Twist get_buf_cmd();
    void setzero_buf_cmd();
    bool set_obj_map (gazebo_msgs::ModelStates obj_map_);
    bool get_is_under_planner_control();
    bool set_target (geometry_msgs::Pose2D target_);
    geometry_msgs::Pose2D get_target();
    bool set_target_name(std::string name);
    std::string get_target_name();
    bool set_position(geometry_msgs::PoseStamped position_);
    geometry_msgs::PoseStamped get_position();
    bool set_closest_as_target (std::string obj_name);
    bool set_need_clear_map(bool val);
    bool get_need_clear_map();
};

////////////////////////////////////////////////////////////////////////////////
//  Utility functions
////////////////////////////////////////////////////////////////////////////////

double distance (double x1, double y1, double x2, double y2);

#endif /* ACTION_SERVER_H_ */
