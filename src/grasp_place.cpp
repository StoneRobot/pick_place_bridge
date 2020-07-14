#include "pick_place_bridge/grasp_place.h"


GraspPlace::GraspPlace(ros::NodeHandle n)
{
    nh = n;
    MoveGroup = new moveit::planning_interface::MoveGroupInterface(GROUP);
    // 发布
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    // 订阅
    detachObjectSub = nh.subscribe("detach_object", 10, &GraspPlace::detachObjectCallback, this);
    // 客户端
    openGripperClient = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    closeGripperClient = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");
    getForceClient = nh.serviceClient<hirop_msgs::getForce>("getForce");
    moveSeqClient = nh.serviceClient<hirop_msgs::moveSeqIndex>("moveSeq");

    // MoveGroup->setGoalPositionTolerance(0.01);
    MoveGroup->setGoalOrientationTolerance(0.05);
} 

GraspPlace::~GraspPlace()
{
    delete MoveGroup;
    MoveGroup = nullptr;
} 

void GraspPlace::detachObjectCallback(const std_msgs::Empty::ConstPtr& msg)
{
    detachRobotObject();
}

void GraspPlace::detachRobotObject()
{
    MoveGroup->detachObject(OBJECT);
}

bool GraspPlace::code2Bool(moveit::planning_interface::MoveItErrorCode code)
{
    ROS_INFO_STREAM(code);
    if(code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO_STREAM("code is SUCCESS");
        return true;
    }
    else
    {
        ROS_INFO_STREAM("code is FAILED");
        return false;
    }
}


bool GraspPlace::setAndMove(geometry_msgs::PoseStamped& targetPose)
{
    setStartState();
    // ROS_INFO_STREAM("setAndMove: " << targetPose);
    MoveGroup->setPoseTarget(targetPose);
    MoveGroup->getPoseTarget();
    ROS_INFO_STREAM(MoveGroup->getPoseTarget());
    bool flag;
    flag = moveGroupPlanAndMove();
    MoveGroup->detachObject(OBJECT);

    return flag;
}

bool GraspPlace::moveGroupPlanAndMove()
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    if(!isStop)  
    do
    {
        code = MoveGroup->plan(my_plan);
        cnt++;
    }
    while (ros::ok() && cnt < 5 && code != moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop);
    if(code == moveit::planning_interface::MoveItErrorCode::SUCCESS && !isStop)
    {
        code = loop_move();
    }
    return code2Bool(code);
}

bool GraspPlace::loop_move()
{
    int cnt = 0;
    moveit::planning_interface::MoveItErrorCode code;
    do
    {
        ROS_INFO_STREAM("loop_move");
        code = MoveGroup->move();
        cnt ++;
    }
    while (ros::ok() && cnt < 5 && code == moveit::planning_interface::MoveItErrorCode::TIMED_OUT && !isStop==false);
    ROS_INFO_STREAM("execution is completed");
    return code2Bool(code);
}

bool GraspPlace::robotMoveCartesianUnit2(double x, double y, double z)
{
    setStartState();

    geometry_msgs::PoseStamped temp_pose = MoveGroup->getCurrentPose();
    MoveGroup->getCurrentState();
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    temp_pose.pose.position.x +=x;
    temp_pose.pose.position.y +=y;
    temp_pose.pose.position.z +=z;
    geometry_msgs::Pose target_pose = temp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    int cnt=0;
    double computeCartesian;
    while(ros::ok())
    {
        computeCartesian = MoveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_STREAM("computeCartesian: " << computeCartesian); 
        if(computeCartesian > 0.75 || isStop || cnt == 5)
        {
            if(cnt == 5)
                return false;
            break;
        }
        ++cnt;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    moveit::planning_interface::MoveItErrorCode code;
    // if(!isStop)
    // {
    //     code = MoveGroup->execute(plan);
    //     return code2Bool(code);
    // }
    cnt = 0;
    do{
        code = MoveGroup->execute(plan);
        if(code2Bool(code))
            return true;
        cnt++;
    }
    while(ros::ok() && !isStop && code != moveit::planning_interface::MoveItErrorCode::SUCCESS && cnt < 20);
    return false;
}

// 使用世界坐标系
bool GraspPlace::robotMoveCartesianUnit2(geometry_msgs::PoseStamped& poseStamped)
{
    ROS_INFO_STREAM("robotMoveCartesianUnit2------------>>pose");
    setStartState();
    // ros::Duration(0.1).sleep();

    geometry_msgs::PoseStamped temp_pose = MoveGroup->getCurrentPose();
    std::vector<geometry_msgs::Pose> waypoints;

    const double jump_threshold = 0;
    const double eef_step = 0.02;

    // waypoints.push_back(temp_pose.pose);
    waypoints.push_back(poseStamped.pose);

    moveit_msgs::RobotTrajectory trajectory;
    int cnt=0;
    double computeCartesian;
    while(ros::ok())
    {
        computeCartesian = MoveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_STREAM("computeCartesian: " << computeCartesian);
        if(computeCartesian >= 0.5 || isStop || cnt == 5)
        {
            if(cnt == 5 ) return false;

            break;
        }
        ++cnt;
    }
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    moveit::planning_interface::MoveItErrorCode code;
    // if(!isStop)
    // {
    //     code = MoveGroup->execute(plan);
    //     return code2Bool(code);
    // }
    cnt = 0;
    do{
        code = MoveGroup->execute(plan);
        if(code2Bool(code))
            return true;
        cnt++;
    }
    while(ros::ok() && !isStop && code != moveit::planning_interface::MoveItErrorCode::SUCCESS && cnt < 20);
    return false;
}

 inline double GraspPlace::angle2rad(double& angle)
 {
    return (angle/180)*M_PI;
 }

 bool GraspPlace::transformFrame(geometry_msgs::PoseStamped& poseStamped, std::string frame_id="world")
{
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::PoseStamped* worldFramePose = new geometry_msgs::PoseStamped[1];
    geometry_msgs::PoseStamped* otherFramePose = new geometry_msgs::PoseStamped[1];
    tf::TransformListener listener;

    otherFramePose[0] = poseStamped;
    for(int i=0; i < 5; ++i)
    {
        try
        {
            listener.transformPose(frame_id, otherFramePose[0], worldFramePose[0]);
            break;
        }
        catch(tf::TransformException& ex)
        {
            ROS_INFO_STREAM(ex.what());
            ros::WallDuration(1).sleep();
            continue;
        }
    }
    poseStamped = worldFramePose[0];
    delete[] worldFramePose;
    delete[] otherFramePose;
    if(poseStamped.header.frame_id == FATHER_FRAME)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void GraspPlace::showObject(geometry_msgs::Pose pose, std::string objectName="object")
{
    std::vector<moveit_msgs::CollisionObject> collisionObject;
    collisionObject.resize(1);
    collisionObject[0].id = objectName;
    collisionObject[0].header.frame_id = FATHER_FRAME;
    collisionObject[0].primitives.resize(1);
    collisionObject[0].primitives[0].type = collisionObject[0].primitives[0].BOX;
    collisionObject[0].primitives[0].dimensions.resize(3);
    collisionObject[0].primitives[0].dimensions[0] = 0.033;
    collisionObject[0].primitives[0].dimensions[1] = 0.033;
    collisionObject[0].primitives[0].dimensions[2] = 0.106;

    collisionObject[0].primitive_poses.resize(1);
    collisionObject[0].primitive_poses[0].position.x = pose.position.x;
    collisionObject[0].primitive_poses[0].position.y = pose.position.y;
    collisionObject[0].primitive_poses[0].position.z = pose.position.z;
    collisionObject[0].primitive_poses[0].orientation.x = pose.orientation.x;
    collisionObject[0].primitive_poses[0].orientation.y = pose.orientation.y;
    collisionObject[0].primitive_poses[0].orientation.z = pose.orientation.z;
    collisionObject[0].primitive_poses[0].orientation.w = pose.orientation.w;
    collisionObject[0].operation = moveit_msgs::CollisionObject::ADD;

    moveit_msgs::PlanningScene p;
    p.world.collision_objects.push_back(collisionObject[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    planning_scene_diff_publisher.publish(p);
    ros::Duration(1).sleep();
}

void GraspPlace::rmObject(std::string name="object")
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = name;
    collision_objects[0].operation = collision_objects[0].REMOVE;
    moveit_msgs::PlanningScene p;
    p.world.collision_objects.push_back(collision_objects[0]);
    p.is_diff = true;
    p.robot_state.is_diff = true;
    planning_scene_diff_publisher.publish(p);
    ros::Duration(1).sleep();
}

bool GraspPlace::setStartState()
{
    std::vector<double> joint = MoveGroup->getCurrentJointValues();
    moveit_msgs::RobotState r;
    r.joint_state.position = joint;
    MoveGroup->setStartState(r);
    MoveGroup->setStartStateToCurrentState();
    return true;
}

bool GraspPlace::increaseTheAccuracyOfQuat(geometry_msgs::PoseStamped& pose)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose.pose.orientation, quat);
    double r, p, y;

    tf::Matrix3x3(quat).setRPY(r, p, y);

    
    tf2::Quaternion orientation;
    orientation.setRPY(y, p, r);
    pose.pose.orientation = tf2::toMsg(orientation);
}

geometry_msgs::PoseStamped GraspPlace::getPreparePose(geometry_msgs::PoseStamped pose, double x)
{
    Eigen::Matrix3d R = Quaternion2RotationMatrix(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    Eigen::Matrix4d TR;
    for(int i=0; i<3; ++i)
        for(int j=0; j<3; ++j)
        {
            TR(i, j) = R(i, j);
        }
    TR(3, 3) = 1;
    TR(0, 3) = pose.pose.position.x;
    TR(1, 3) = pose.pose.position.y;
    TR(2, 3) = pose.pose.position.z;
    Eigen::Matrix4d T = Eigen::MatrixXd::Identity(4, 4);
    T(0, 3) = -x;
    auto targetT = TR*T;
    ROS_INFO_STREAM("\n" << targetT);
    pose.pose.position.x = targetT(0, 3);
    pose.pose.position.y = targetT(1, 3);
    pose.pose.position.z = targetT(2, 3);
    ROS_INFO_STREAM("getPreparePose: " << pose);
    return pose;
}

Eigen::Matrix3d GraspPlace::Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;  
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();  
    ROS_INFO_STREAM("Quaternion2RotationMatrix result is: \n" << R);
    return R;  
} 

bool GraspPlace::pick(geometry_msgs::PoseStamped pose, double pre_grasp_approach=0.1, double post_grasp_retreat=0.1)
{
    geometry_msgs::PoseStamped preparePose;

    // rmObject(OBJECT);
    // showObject(pose.pose);

    // increaseTheAccuracyOfQuat(pose);


    fiveFightGripperPoseIndex(HOME);
    ros::Duration(2).sleep();
    preparePose = getPreparePose(pose, pre_grasp_approach);
    ROS_INFO_STREAM("preparePose: " << preparePose);
    if(setAndMove(preparePose))
    {
        if(robotMoveCartesianUnit2(pose))
        {
            fiveFightGripperPoseIndex(GRASP);
            ros::Duration(3).sleep();
            // MoveGroup->attachObject(OBJECT);
            if(robotMoveCartesianUnit2(0, 0, 0.02))
            {
                geometry_msgs::PoseStamped nowPose = MoveGroup->getCurrentPose();
                geometry_msgs::PoseStamped endPose;
                endPose = getPreparePose(nowPose, post_grasp_retreat);
                if(robotMoveCartesianUnit2(endPose))
                {
                    ROS_INFO_STREAM("Execute successfully");
                    return true;
                }
            }
        }
    }
    if(isStop)
        isStop = false;
    ROS_INFO_STREAM("Execute failed");
    return false;
}


bool GraspPlace::place(geometry_msgs::PoseStamped pose, double pre_place_approach=0.75, double post_place_retreat=0.75)
{
    // increaseTheAccuracyOfQuat(pose);
    geometry_msgs::PoseStamped preparePose;
    preparePose = getPreparePose(pose, pre_place_approach);
    ROS_WARN_STREAM(" getPreparePose ...");
    if(setAndMove(preparePose))
    {
        if(robotMoveCartesianUnit2(pose))
        {
            
            fiveFightGripperPoseIndex(HOME);
            ros::Duration(1).sleep();
            // MoveGroup->detachObject(OBJECT);
            // if(robotMoveCartesianUnit2(0, 0, 0.01))
            // {
                geometry_msgs::PoseStamped nowPose = MoveGroup->getCurrentPose();
                geometry_msgs::PoseStamped endPose;
                endPose = getPreparePose(nowPose, post_place_retreat);
                if(robotMoveCartesianUnit2(endPose))
                {
                    ROS_INFO_STREAM("Execute successfully");
                    return true;
                }
            // }
        }
    }
    if(isStop)
        isStop = false;
    ROS_INFO_STREAM("Execute failed");
    return false;
}

bool GraspPlace::fixedPick(geometry_msgs::PoseStamped pose, double pre_grasp_approach, double post_grasp_retreat)
{
    ROS_INFO_STREAM("---------------pick fixed orientation---------------");
    rmObject(OBJECT);
    tf2::Quaternion q;
    q.setRPY(0, 0, -M_PI/2);
    pose.pose.orientation = tf2::toMsg(q);
    showObject(pose.pose);
    pose.pose.position.y += 0.1;
    //////////////////
    // geometry_msgs::PoseStamped movePose;
    // movePose.pose.position.x = 0.0553143;
    // movePose.pose.position.y = -0.858108;
    // movePose.pose.position.z = 1.55026;

    // movePose.pose.orientation.x = 0.248138;
    // movePose.pose.orientation.y = 0.312854;
    // movePose.pose.orientation.z = -0.590081;
    // movePose.pose.orientation.w = 0.70168;
    setAndMove(pose);
    //////////
    fiveFightGripperPoseIndex(HOME);
    robotMoveCartesianUnit2(0, -0.1, 0);

    fiveFightGripperPoseIndex(GRASP);

    MoveGroup->attachObject(OBJECT);
    robotMoveCartesianUnit2(0, 0, 0.01);

    robotMoveCartesianUnit2(0, 0.1, 0);
    ROS_INFO_STREAM("---------------pick fixed orientation over---------------");
    if(isStop)
        isStop = false;
    return true;
}

bool GraspPlace::fixedPlace(double y)
{
    geometry_msgs::PoseStamped preparePose;
    preparePose.header.frame_id = FATHER_FRAME;
    preparePose.pose.position.x = 0.86;
    preparePose.pose.position.y = y;
    preparePose.pose.position.z = 1.4;
    preparePose.pose.orientation.w = 1;
    setAndMove(preparePose);

    robotMoveCartesianUnit2(0.1, 0, 0);

    fiveFightGripperPoseIndex(HOME);
    MoveGroup->detachObject(OBJECT);
    robotMoveCartesianUnit2(0, 0, 0.01);
    
    robotMoveCartesianUnit2(-0.1, 0, 0);
    if(isStop)
        isStop = false;
    return true;
}

bool GraspPlace::move(geometry_msgs::PoseStamped& pose)
{
    bool flag;
    flag = setAndMove(pose);
    if(isStop)
        isStop = false;
    return flag;
}

bool GraspPlace::backHome()
{
    ROS_INFO_STREAM("----back home ...----");
    bool result=false;
    if(!isStop)
    {
        MoveGroup->setNamedTarget(HOME_POSE);
        result = (MoveGroup->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    fiveFightGripperPoseIndex(HOME);
    ROS_INFO_STREAM("----back home end----");
    if(isStop)
        isStop=false;
    return result;
}

bool GraspPlace::stop()
{
    isStop = true;
    MoveGroup->stop();
    return true;
}

bool GraspPlace::openGripper()
{
    hirop_msgs::openGripper srv;
    ROS_INFO_STREAM("open gripper ...");
    if(openGripperClient.call(srv))
    {
        if(srv.response.isOpen)
            ROS_INFO_STREAM("opened gripper");
        else
            ROS_INFO_STREAM("open gripper faild");
        return srv.response.isOpen;
    }
    ROS_INFO_STREAM("check gripper server");
    return false;
}

bool GraspPlace::closeGripper()
{
    hirop_msgs::closeGripper srv;
    ROS_INFO_STREAM("close gripper ...");
    if(closeGripperClient.call(srv))
    {
        if(srv.response.isClose)
            ROS_INFO_STREAM("closed gripper");
        else
            ROS_INFO_STREAM("close gripper faild");
        return srv.response.isClose;
    }
    ROS_INFO_STREAM("check gripper server");
    return false;
}

bool GraspPlace::fiveFightGripperPoseIndex(int index)
{
    ROS_INFO_STREAM("Pose index: " << index);
    hirop_msgs::moveSeqIndex srv;
    srv.request.index = index;
    if(!isStop)
        if(moveSeqClient.call(srv))
        {
            if(srv.response.sucesss)
                ROS_INFO_STREAM("alreay move to pose");
            else
                ROS_INFO_STREAM("move to pose faild");
            return srv.response.sucesss;
        }
        else
            ROS_INFO_STREAM("check gripper server");
    return false;
}

bool GraspPlace::speedScale(float scale)
{

    ROS_INFO_STREAM("Velocity Scaling Factor: " << scale);
    MoveGroup->setMaxVelocityScalingFactor(scale);
    MoveGroup->setMaxVelocityScalingFactor(scale);
    return true;
}

void GraspPlace::setStopFlag(bool flag)
{
    isStop = flag;
}

geometry_msgs::PoseStamped GraspPlace::getNowPose()
{
    setStartState();
    return MoveGroup->getCurrentPose();
}

