#include "pick_place_bridge/grasp_place.h"
#include "pick_place_bridge/PickPlacePose.h"
#include "rb_msgAndSrv/rb_DoubleBool.h"


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
	
	// 话题回调
    void backHomeCallback(const std_msgs::Int8::ConstPtr& msg);
    void sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg);
    void speedScaleCallback(const std_msgs::Bool::ConstPtr& msg);
    
	// 服务器
    ros::ServiceServer pickServer;
    ros::ServiceServer placeServer;
    ros::ServiceServer fixedPickServer;
    ros::ServiceServer fixedPlaceServer;
    ros::ServiceServer moveServer;
    ros::ServiceServer handgestureServer;
	// 订阅
    ros::Subscriber stopMoveSub;
    ros::Subscriber backHomeSub;
    ros::Subscriber speedScaleSub;
    

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
	
	// 话题
    backHomeSub = nh.subscribe("/back_home", 1, &Grasp::backHomeCallback, this);
    stopMoveSub = nh.subscribe("/stop_move", 1, &Grasp::sotpMoveCallback, this);
    speedScaleSub = nh.subscribe("/speedScale", 10, &Grasp::speedScaleCallback, this);
    
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
    setAndMove(req.Pose);
    return rep.result;
}



void Grasp::backHomeCallback(const std_msgs::Int8::ConstPtr& msg)
{
    backHome();
}

void Grasp::sotpMoveCallback(const std_msgs::Bool::ConstPtr& msg)
{
    stop();
}

void Grasp::speedScaleCallback(const std_msgs::Bool::ConstPtr& msg)
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
