#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotState.h>

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/moveSeqIndex.h"
#include "hirop_msgs/getForce.h"

class GraspPlace
{
public:
    GraspPlace(ros::NodeHandle n);
    virtual ~GraspPlace();

    bool setAndMove(geometry_msgs::PoseStamped& targetPose);
    bool moveGroupPlanAndMove();
    bool loop_move();

    bool robotMoveCartesianUnit2(double x, double y, double z);
    bool robotMoveCartesianUnit2(geometry_msgs::PoseStamped& poseStamped);
    double angle2rad(double& angle);
    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);
    void showObject(geometry_msgs::Pose pose);
    void rmObject(std::string name);
    bool setStartState();
    bool code2Bool(moveit::planning_interface::MoveItErrorCode code);
    geometry_msgs::PoseStamped getPreparePose(geometry_msgs::PoseStamped pose, double x);
    Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);
    bool pick(geometry_msgs::PoseStamped pose, double pre_grasp_approach, double post_grasp_retreat);
    bool place(geometry_msgs::PoseStamped pose, double pre_place_approach, double post_place_retreat);
    bool fixedPick(geometry_msgs::PoseStamped pose, double pre_grasp_approach, double post_grasp_retreat);
    bool fixedPlace(double y);
    bool backHome();
    bool speedScale(bool isSlow);
    bool stop();
    bool openGripper();
    bool closeGripper();
    bool fiveFightGripperPoseIndex(int index);
    void detachRobotObject();
    void setStopFlag(bool flag);
    geometry_msgs::PoseStamped getNowPose();
private:
    void detachObjectCallback(const std_msgs::Empty::ConstPtr& msg);
    // 发布
    // 用于发布OBJECT
    ros::Publisher planning_scene_diff_publisher;
    // 订阅
    // 从机器人上移除OBJECT
    ros::Subscriber detachObjectSub;
    // 客户端
    // 二指
    ros::ServiceClient openGripperClient;
    ros::ServiceClient closeGripperClient;
    // 五指
    ros::ServiceClient getForceClient;
    ros::ServiceClient moveSeqClient;

    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface* MoveGroup;
    bool isStop=false;

    // 常量
    const std::string OBJECT = "object";
    const std::string GROUP = "arm";
    const std::string HOME_POSE = "home";
    const std::string FATHER_FRAME = "world";
    const int SHAKE = 1;
    const int GRASP = 2;
    const int OK = 3;
    const int HOME = 4;
    const int SHAKE_PREPARE = 5;
};
 