#include "pick_place_bridge/grasp_place.h"
#include "pick_place_bridge/PickPlacePose.h"
#include "pick_place_bridge/recordPose.h"
#include "pick_place_bridge/SpeedScale.h"
#include "rb_msgAndSrv/rb_DoubleBool.h"
#include <std_srvs/Empty.h>
#include <std_msgs/Float32.h>


class Grasp : public GraspPlace
{
public:
    Grasp(ros::NodeHandle n);
private:	
    ros::NodeHandle nh;
	// 服务回调
    bool pickCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep);
    bool placeCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep);
    bool fixedPickCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep);
    bool fixedPlaceCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep);
    bool moveCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep);
    bool recerdPoseCallback(pick_place_bridge::recordPose::Request& req, pick_place_bridge::recordPose::Response& rep);
    bool backHomeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);
    bool setSpeedScaleCallback(pick_place_bridge::SpeedScale::Request& req, pick_place_bridge::SpeedScale::Response& rep);
    bool stopMoveCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep);
	
	// 话题回调
    void stopMoveCallback(const std_msgs::Bool::ConstPtr& msg);
    void startMoveCallback(const std_msgs::Bool::ConstPtr& msg);
    void speedScaleCallback(const std_msgs::Float32::ConstPtr& msg);
    void isSetConstraintCallback(const std_msgs::Bool::ConstPtr& msg);
    
	// 服务器
    ros::ServiceServer pickServer;
    ros::ServiceServer placeServer;
    ros::ServiceServer fixedPickServer;
    ros::ServiceServer fixedPlaceServer;
    ros::ServiceServer moveServer;
    ros::ServiceServer handgestureServer;
    ros::ServiceServer recerdPoseServer;
    ros::ServiceServer backHomeServer;
    ros::ServiceServer setSpeedScaleServer;
    ros::ServiceServer stopMoveServer;
	// 订阅
    ros::Subscriber stopMoveSub;
    ros::Subscriber startMoveSub;
    ros::Subscriber speedScaleSub;
    ros::Subscriber isSetConstraintSub;
};

Grasp::Grasp(ros::NodeHandle n)
:GraspPlace(n)
{
    nh = n;
	//	服务
    pickServer = nh.advertiseService("pick", &Grasp::pickCallback, this);
    placeServer = nh.advertiseService("place", &Grasp::placeCallback, this);
    fixedPickServer = nh.advertiseService("fixed_pick", &Grasp::fixedPickCallback, this);
    fixedPlaceServer = nh.advertiseService("fixed_place", &Grasp::fixedPlaceCallback, this);
    moveServer = nh.advertiseService("move", &Grasp::moveCallback, this);
	recerdPoseServer = nh.advertiseService("recordPose", &Grasp::recerdPoseCallback, this);
    backHomeServer = nh.advertiseService("/back_home", &Grasp::backHomeCallback, this);
    setSpeedScaleServer = nh.advertiseService("/set_speed_scale", &Grasp::setSpeedScaleCallback, this);
    stopMoveServer = nh.advertiseService("stopMove", &Grasp::stopMoveCallback, this);
	// 话题
    stopMoveSub = nh.subscribe("/stop_move", 1, &Grasp::stopMoveCallback, this);
    startMoveSub = nh.subscribe("start_move", 1, &Grasp::startMoveCallback, this);
    speedScaleSub = nh.subscribe("/speedScale", 10, &Grasp::speedScaleCallback, this);
    isSetConstraintSub = nh.subscribe("/is_setConstrain", 10, &Grasp::isSetConstraintCallback, this);
    
};

bool Grasp::pickCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep)
{
    rep.result = pick(req.Pose, 0.1, 0.1);
    return rep.result;
}

bool Grasp::placeCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep)
{
    rep.result = place(req.Pose, 0.1, 0.1);
    return rep.result;
}

bool Grasp::fixedPickCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep)
{
    fixedPick(req.Pose, 0.1, 0.1);
    rep.result = true;
    return true;
}

bool Grasp::fixedPlaceCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep)
{
    fixedPlace(req.Pose.pose.position.y);
    rep.result = true;
    return true;
}

bool Grasp::moveCallback(pick_place_bridge::PickPlacePose::Request& req, pick_place_bridge::PickPlacePose::Response& rep)
{
    rep.result = move(req.Pose);
    return rep.result;
}

bool Grasp::recerdPoseCallback(pick_place_bridge::recordPose::Request& req, pick_place_bridge::recordPose::Response& rep)
{
    rep.pose = getNowPose();
    return true;
}

bool Grasp::setSpeedScaleCallback(pick_place_bridge::SpeedScale::Request& req, pick_place_bridge::SpeedScale::Response& rep)
{
    speedScale(req.scale);
    rep.result = true;
    return false;
}

bool Grasp::stopMoveCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
    stop();
}

void Grasp::stopMoveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
        stop();
}

void Grasp::startMoveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
        setStopFlag(false);
}

void Grasp::isSetConstraintCallback(const std_msgs::Bool::ConstPtr& msg)
{
    setConStraintFlag(msg->data);
}

bool Grasp::backHomeCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& rep)
{
    backHome();
}


void Grasp::speedScaleCallback(const std_msgs::Float32::ConstPtr& msg)
{
    speedScale(msg->data);
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pick_place_bridge");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    Grasp g(nh);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
