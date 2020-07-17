#pragma once
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/joint_model_group.h>

#include <math.h>

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

#include <iostream>

#include <vector>

#include "hirop_msgs/openGripper.h"
#include "hirop_msgs/closeGripper.h"
#include "hirop_msgs/moveSeqIndex.h"
#include "hirop_msgs/getForce.h"

class GraspPlace
{
public:
    GraspPlace(ros::NodeHandle n);
    virtual ~GraspPlace();

    /**
     * @brief 移动到点
     * @param targetPose 目标点位
    */
    bool setAndMove(geometry_msgs::PoseStamped& targetPose);

    /**
     * @brief 进行直线移动,参考坐标系为世界坐标系
     * @param x 世界坐标X-轴
     * @param y 世界坐标Y-轴
     * @param z 世界坐标Z-轴
    */
    bool robotMoveCartesianUnit2(double x, double y, double z);

    /**
     * @brief 进行直线移动,参考坐标系为世界坐标系
     * @param poseStamped 目标坐标
    */
    bool robotMoveCartesianUnit2(geometry_msgs::PoseStamped& poseStamped);

    /**
     * @brief 显示障碍物, 参考坐标系为世界坐标系
     * @param pose 生成物坐标
    */
    void showObject(geometry_msgs::Pose pose, std::string objectName);

    /**
     * @brief 移除生成的障碍物
     * @brief name 默认为object
    */
    void rmObject(std::string name);

    /**
     * @brief 抓取, 包含物体的姿态
     * @param pose 物体的位置
     * @param pre_grasp_approach 抓取的寸进值, 默认为0.1
     * @param post_grasp_retreat 退出的寸进值, 默认为0.1
    */
    bool pick(geometry_msgs::PoseStamped pose, double pre_grasp_approach, double post_grasp_retreat);

    /**
     * @brief 放置, 包含物体的姿态
     * @param pose 放置物体的位置
     * @param pre_grasp_approach 放置的寸进值, 默认为0.1
     * @param post_grasp_retreat 退出的寸进值, 默认为0.1
    */
    bool place(geometry_msgs::PoseStamped pose, double pre_place_approach, double post_place_retreat);

    /**
     * @brief 抓取, 固定的方向
     * @param pose 物体的位置
     * @param pre_grasp_approach 抓取的寸进值, 默认为0.1
     * @param post_grasp_retreat 退出的寸进值, 默认为0.1
    */
    bool fixedPick(geometry_msgs::PoseStamped pose, double pre_grasp_approach, double post_grasp_retreat);

    /**
     * @brief 放置, 包含物体的姿态
     * @param y 放置物体的位置
    */
    bool fixedPlace(double y);

    /**
     * @brief 回Home点
    */
    bool backHome();

    /**
     * @brief 调速
     * @param scale 百分比
    */
    bool speedScale(float scale);

    /**
     * @brief 停止运动
    */
    bool stop();

    /**
     * @brief 打开二指夹爪
    */
    bool openGripper();

    /**
     * @brief 关闭二指夹爪
    */
    bool closeGripper();

    /**
     * @brief 五指夹爪动作索引
     * @param index 动作索引
    */
    bool fiveFightGripperPoseIndex(int index);

    /**
     * @brief 移除机器人上的object
    */
    void detachRobotObject();

    /**
     * @brief 机器人停止
     * @param flag true为停止
    */
    void setStopFlag(bool flag);

    /**
     * @brief 得到现在机器人的点位
    */
    geometry_msgs::PoseStamped getNowPose();

    bool increaseTheAccuracyOfQuat(geometry_msgs::PoseStamped& pose);

    bool move(geometry_msgs::PoseStamped& pose);
    bool setConstraint();
    bool clearConstraints();
    double chooseAngle(std::vector<double> joint, double currentJoint);
    void setConStraintFlag(bool flag);
    void pubStatus(bool isBUSY);
private:
    geometry_msgs::PoseStamped getPreparePose(geometry_msgs::PoseStamped pose, double x);
    Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w);
    bool code2Bool(moveit::planning_interface::MoveItErrorCode code);
    /**
     * @brief 
    */
    bool setStartState();

    bool transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id);

    double angle2rad(double& angle);

    bool moveGroupPlanAndMove();
    bool loop_move();

    void detachObjectCallback(const std_msgs::Empty::ConstPtr& msg);
    /****** 发布 ******/
    // 用于发布OBJECT
    ros::Publisher planning_scene_diff_publisher;
    ros::Publisher freeStatusPub;
    // 订阅
    // 从机器人上移除OBJECT
    ros::Subscriber detachObjectSub;
    
    /****** 客户端 ******/
    // 二指
    ros::ServiceClient openGripperClient;
    ros::ServiceClient closeGripperClient;
    // 五指
    ros::ServiceClient getForceClient;
    ros::ServiceClient moveSeqClient;

    ros::NodeHandle nh;
    moveit::planning_interface::MoveGroupInterface* MoveGroup;
    bool isStop=false;
    bool isSetConstraint;

    robot_model::RobotModelPtr robotModelPtr;
    robot_state::RobotStatePtr robotStatePtr;
    robot_state::JointModelGroup* jointModelGroupPtr;

    // 常量
    const std::string OBJECT = "object";
    const std::string GROUP = "arm";
    const std::string HOME_POSE = "home";
    const std::string FATHER_FRAME = "world";
    // const int SHAKE = 1;
    const int GRASP = 2;
    const int OK = 3;
    const int HOME = 4;
    const int SHAKE_PREPARE = 5;
};
 