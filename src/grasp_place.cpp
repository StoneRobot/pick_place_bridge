#include "pick_place_bridge/grasp_place.h"


GraspPlace::GraspPlace(ros::NodeHandle n)
{
    nh = n;
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    MoveGroup = new moveit::planning_interface::MoveGroupInterface(GROUP);
    openGripperClient = nh.serviceClient<hirop_msgs::openGripper>("openGripper");
    closeGripperClient = nh.serviceClient<hirop_msgs::closeGripper>("closeGripper");
} 

GraspPlace::~GraspPlace()
{
    delete MoveGroup;
    MoveGroup = nullptr;
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

void GraspPlace::move(geometry_msgs::PoseStamped& targetPose)
{
    setAndMove(targetPose);
}

moveit::planning_interface::MoveItErrorCode GraspPlace::setAndMove(geometry_msgs::PoseStamped& targetPose)
{
    setStartState();
    ROS_INFO_STREAM("setAndMove: " << targetPose);
    MoveGroup->setPoseTarget(targetPose);
    this->moveGroupPlanAndMove();
}

moveit::planning_interface::MoveItErrorCode GraspPlace::moveGroupPlanAndMove()
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
    return code;
}

moveit::planning_interface::MoveItErrorCode GraspPlace::loop_move()
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
    return code;
}

bool GraspPlace::robotMoveCartesianUnit2(double x, double y, double z)
{
    setStartState();

    geometry_msgs::PoseStamped temp_pose = MoveGroup->getCurrentPose();

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    temp_pose.pose.position.x +=x;
    temp_pose.pose.position.y +=y;
    temp_pose.pose.position.z +=z;
    geometry_msgs::Pose target_pose = temp_pose.pose;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    while( MoveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 1 && !isStop && ros::ok());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    moveit::planning_interface::MoveItErrorCode code;
    if(!isStop)
    {
        code = MoveGroup->execute(plan);
        return code2Bool(code);
    }
    return false;
}

// 使用世界坐标系
bool GraspPlace::robotMoveCartesianUnit2(geometry_msgs::PoseStamped& poseStamped)
{
    setStartState();

    geometry_msgs::PoseStamped temp_pose = MoveGroup->getCurrentPose();
    std::vector<geometry_msgs::Pose> waypoints;

    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    waypoints.push_back(temp_pose.pose);
    waypoints.push_back(poseStamped.pose);

    moveit_msgs::RobotTrajectory trajectory;
    while( MoveGroup->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory) < 0.8 && !isStop && ros::ok());
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    moveit::planning_interface::MoveItErrorCode code;
    if(!isStop)
    {
        code = MoveGroup->execute(plan);
        return code2Bool(code);
    }
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
    if(poseStamped.header.frame_id == "world")
    {
        return true;
    }
    else
    {
        return false;
    }
}

void GraspPlace::showObject(geometry_msgs::Pose pose)
{
    std::vector<moveit_msgs::CollisionObject> collisionObject;
    collisionObject.resize(1);
    collisionObject[0].id = OBJECT;
    collisionObject[0].header.frame_id = "world";
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
    rmObject(OBJECT);
    showObject(pose.pose);
    preparePose = getPreparePose(pose, pre_grasp_approach);
    ROS_INFO_STREAM("preparePose: " << preparePose);
    setAndMove(preparePose);

    openGripper();
    robotMoveCartesianUnit2(pose);

    closeGripper();
    MoveGroup->attachObject(OBJECT);
    robotMoveCartesianUnit2(0, 0, 0.01);

    geometry_msgs::PoseStamped nowPose = MoveGroup->getCurrentPose();
    geometry_msgs::PoseStamped endPose;
    endPose = getPreparePose(nowPose, post_grasp_retreat);
    robotMoveCartesianUnit2(endPose);

    return true;
}

bool GraspPlace::fixedPick(geometry_msgs::PoseStamped pose, double pre_grasp_approach, double post_grasp_retreat)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, -M_PI/2);
    pose.pose.orientation = tf2::toMsg(q);
    pose.pose.position.y += 0.1;
    setAndMove(pose);

    openGripper();
    robotMoveCartesianUnit2(0, -0.1, 0);

    closeGripper();
    MoveGroup->attachObject(OBJECT);
    robotMoveCartesianUnit2(0, 0, 0.01);

    robotMoveCartesianUnit2(0, 0.1, 0);
    return true;
}

bool GraspPlace::place(geometry_msgs::PoseStamped pose, double pre_place_approach=0.1, double post_place_retreat=0.1)
{
    geometry_msgs::PoseStamped preparePose;
    preparePose = getPreparePose(pose, pre_place_approach);
    setAndMove(preparePose);
    robotMoveCartesianUnit2(pose);

    openGripper();
    MoveGroup->detachObject(OBJECT);
    robotMoveCartesianUnit2(0, 0, 0.01);

    geometry_msgs::PoseStamped nowPose = MoveGroup->getCurrentPose();
    geometry_msgs::PoseStamped endPose;
    endPose = getPreparePose(nowPose, post_place_retreat);
    robotMoveCartesianUnit2(endPose);
    return true;
}

bool GraspPlace::fixedPlace(double y)
{
    geometry_msgs::PoseStamped preparePose;
    preparePose.header.frame_id = "world";
    preparePose.pose.position.x = 0.86;
    preparePose.pose.position.y = y;
    preparePose.pose.position.z = 1.4;
    preparePose.pose.orientation.w = 1;
    setAndMove(preparePose);

    robotMoveCartesianUnit2(0.1, 0, 0);

    openGripper();
    MoveGroup->detachObject(OBJECT);
    robotMoveCartesianUnit2(0, 0, 0.01);
    
    robotMoveCartesianUnit2(-0.1, 0, 0);
    return true;
}

bool GraspPlace::backHome()
{
    MoveGroup->setNamedTarget("home");
    MoveGroup->move();
}

bool GraspPlace::stop()
{
    isStop = true;
    MoveGroup->stop();
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

bool GraspPlace::speedScale(bool isSlow)
{
    if(isSlow)
    {
        MoveGroup->setMaxVelocityScalingFactor(0.1);
        MoveGroup->setMaxVelocityScalingFactor(0.1);
    }
    else
    {
        MoveGroup->setMaxVelocityScalingFactor(1);
        MoveGroup->setMaxVelocityScalingFactor(1);
    }
    return true;
}

bool GraspPlace::handgesture()
{
    geometry_msgs::PoseStamped targetPose;
    targetPose.header.frame_id = "world";
    targetPose.pose.position.x = 0.80;
    targetPose.pose.position.y = 0.0;
    targetPose.pose.position.z = 1.4;
    targetPose.pose.orientation.w = 1;
    move(targetPose);
    return true;
}