#if 1
  #define ROS_DEBUG_TEST_01 0 // teb_path data 출력
  #define ROS_DEBUG_TEST_02 0 // teb_path.pose data 확인
  #define ROS_DEBUG_TEST_03 0 // teb_path.pose -> base_link 좌표변환 데이터 확인 
  #define ROS_DEBUG_TEST_04 0 // teb_path.pose의 각 벡터간 거리 체크
  #define ROS_DEBUG_TEST_05 0 // 로봇의 현재 pose 확인(로봇 기준)
  #define ROS_DEBUG_TEST_06 1 // 목표점 거리 허용 오차 범위내 도착 로그
  #define ROS_DEBUG_TEST_07 1 // 목표점 헤딩 허용 오차 범위내 도착 로그 
  #define ROS_DEBUG_TEST_08 0 // 로봇이 목표방향으로 돌려야 하는 헤딩 각도(rad)
  #define ROS_DEBUG_TEST_09 0 // 로봇위치 기준 목표점과의 거리(distance)
  #define ROS_DEBUG_TEST_10 0 // teb_path 변환 roll, pitch, yaw 쿼터니언 
  #define ROS_DEBUG_TEST_11 1 // interval time 체크 
  #define ROS_DEBUG_ERROR_01 0 // teb_path.pose TF 변환 에러
  #define ROS_DEBUG_ERROR_02 1 // Tolerance GOAL Distance OK,Tolerance GOAL Yaw NG
  #define ROS_DEBUG_ERROR_03 1 // Tolerance GOAL Distance NG
#else
  #define ROS_DEBUG_TEST_01 1 // teb_path data 출력
  #define ROS_DEBUG_TEST_02 1 // teb_path.pose data 확인
  #define ROS_DEBUG_TEST_03 1 // teb_path.pose -> base_link 좌표변환 데이터 확인 
  #define ROS_DEBUG_TEST_04 1 // teb_path.pose의 각 벡터간 거리 체크
  #define ROS_DEBUG_TEST_05 1 // 로봇의 현재 pose 확인(로봇 기준)
  #define ROS_DEBUG_TEST_06 1 // 목표점 거리 허용 오차 범위내 도착 로그
  #define ROS_DEBUG_TEST_07 1 // 목표점 헤딩 허용 오차 범위내 도착 로그 
  #define ROS_DEBUG_TEST_08 1 // 로봇이 목표방향으로 돌려야 하는 헤딩 각도(rad)
  #define ROS_DEBUG_TEST_09 1 // 로봇위치 기준 목표점과의 거리(distance)
  #define ROS_DEBUG_TEST_10 1 // teb_path 변환 roll, pitch, yaw 쿼터니언 
  #define ROS_DEBUG_TEST_11 1 // interval time 체크 
  #define ROS_DEBUG_ERROR_01 1 // teb_path.pose TF 변환 에러
  #define ROS_DEBUG_ERROR_02 1 // Tolerance GOAL Distance OK,Tolerance GOAL Yaw NG
  #define ROS_DEBUG_ERROR_03 1 // Tolerance GOAL Distance NG  
#endif

#define ROS_DEBUG_ERROR_01 0 // teb_path.pose TF 변환 에러
#define ROS_DEBUG_ERROR_01 0 // teb_path.pose TF 변환 에러
#define ROS_DEBUG_ERROR_01 0 // teb_path.pose TF 변환 에러


#define TFOI_2020_VER "0.90d"


#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"


// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace teb_local_planner
{
  

TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                           costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                           dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), no_infeasible_plans_(0),
                                           last_preferred_rotdir_(RotType::none), initialized_(false), need_rotate_(true)
{
}


TebLocalPlannerROS::~TebLocalPlannerROS()
{
}

void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
  follow_cfg_.reconfigure(config);

  ros::NodeHandle nh("~/" + name_);
  // lock the config mutex externally
  boost::mutex::scoped_lock lock(cfg_.configMutex());

  // create robot footprint/contour model for optimization
  cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
  planner_->updateRobotModel(cfg_.robot_model);

}

void TebLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  // check if the plugin is already initialized
  if(!initialized_)
  {	
    ROS_INFO("TFOI_2020_VER : %s", TFOI_2020_VER);
    name_ = name;
    // create Node Handle with name of plugin (as used in move_base for loading)
    ros::NodeHandle nh("~/" + name);
    ros::NodeHandle nh1("~");
    
	        
    // get parameters of TebConfig via the nodehandle and override the default config
    cfg_.loadRosParamFromNodeHandle(nh);       
    
    // reserve some memory for obstacles
    obstacles_.reserve(500);
        
    // create visualization instance	
    visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_)); 
        
    // create robot footprint/contour model for optimization
    cfg_.robot_model = getRobotFootprintFromParamServer(nh, cfg_);
    
    // create the planner instance
    if (cfg_.hcp.enable_homotopy_class_planning)
    {
      planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies enabled.");
    }
    else
    {
      planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, visualization_, &via_points_));
      ROS_INFO("Parallel planning in distinctive topologies disabled.");
    }
    
    // init other variables
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.
    
    costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

    global_frame_ = costmap_ros_->getGlobalFrameID();
    cfg_.map_frame = global_frame_; // TODO
    robot_base_frame_ = costmap_ros_->getBaseFrameID();

    //Initialize a costmap to polygon converter
    if (!cfg_.obstacles.costmap_converter_plugin.empty())
    {
      try
      {
        costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
        std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
        // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
        boost::replace_all(converter_name, "::", "/");
        costmap_converter_->setOdomTopic(cfg_.odom_topic);
        costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
        costmap_converter_->setCostmap2D(costmap_);
        
        costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
        ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");        
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
        costmap_converter_.reset();
      }
    }
    else 
      ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");
  
    
    // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);    
    
    // init the odom helper to receive the robot's velocity from odom messages
    odom_helper_.setOdomTopic(cfg_.odom_topic);

    // setup dynamic reconfigure
    dynamic_recfg_ = boost::make_shared< dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig> >(nh);
    dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
    dynamic_recfg_->setCallback(cb);
    
    // validate optimization footprint and costmap footprint
    validateFootprints(cfg_.robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);
        
    // setup callback for custom obstacles
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

    // setup callback for custom via-points
    via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);

    local_plan_sub = nh1.subscribe("/move_base/TebLocalPlannerROS/local_plan", 1,&TebLocalPlannerROS::localPlanCallback, this);
    teb_poses_sub = nh1.subscribe("/move_base/TebLocalPlannerROS/teb_poses", 1, &TebLocalPlannerROS::tebPosesCallback, this);
    
    goal_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1,&TebLocalPlannerROS::goalCB, this);
    type_sub_ = nh.subscribe<std_msgs::Int32>("type", 1,&TebLocalPlannerROS::typeCB, this);
    path_subscribe_ = nh.subscribe<nav_msgs::Path>("recorded_path", 10, &TebLocalPlannerROS::receivePathCB, this);

	  angular_sub_ = nh.subscribe<std_msgs::Float32>("angular", 1,&TebLocalPlannerROS::angularCB, this);
    linear_sub_ = nh.subscribe<std_msgs::Float32>("linear", 1,&TebLocalPlannerROS::linearCB, this);

    carrot_pub_ = nh1.advertise<geometry_msgs::PointStamped>("lookahead_point", 1);
    carrot_arc_pub_ = nh.advertise<nav_msgs::Path>("lookahead_collision_arc", 1);


    // initialize failure detector
    ros::NodeHandle nh_move_base("~");
    double controller_frequency = 5;
    nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
    failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration*controller_frequency));
    
    // set initialized flag
    initialized_ = true;

    ROS_DEBUG("teb_local_planner plugin initialized.");
  }
  else
  {
    ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
  }
}
//--------------------------------------------------- Bring up Data Function ----------------------------------------------------------

bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  // check if plugin is initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // store the global plan
  global_plan_.clear();
  global_plan_ = orig_global_plan;

  // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
  // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.  
            
  // reset goal_reached_ flag
  goal_reached_ = false;
  flag_distance = false;
  
  return true;
}

void TebLocalPlannerROS::receivePathCB(const nav_msgs::Path::ConstPtr &msg) 
{
	path_ = *msg;
}

void TebLocalPlannerROS::goalCB(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
	goal_ = * pose;
	get_goal_ = 1;
	get_new_goal_ = 1;
}

void TebLocalPlannerROS::typeCB(const std_msgs::Int32::ConstPtr &msg)
{
	type_ = msg->data;
}

void TebLocalPlannerROS::angularCB(const std_msgs::Float32::ConstPtr &msg)
{
	chiss_angular_ = msg->data;
}

void TebLocalPlannerROS::linearCB(const std_msgs::Float32::ConstPtr &msg)
{
	chiss_linear_ = msg->data;
}

void TebLocalPlannerROS::tebPosesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    // Clear the previous path
    teb_path.poses.clear();

    // Populate the nav_msgs::Path with data from Teb poses
    for(const auto& pose : msg->poses)
    {
        geometry_msgs::PoseStamped path_pose;
        path_pose.pose = pose;
        teb_path.poses.push_back(path_pose);
    }

    // Additional header information for the path message (if needed)
    teb_path.header.stamp = ros::Time::now();
    teb_path.header.frame_id = "map"; // Set your desired frame id
    
    // Use the populated teb_path as needed
}

void TebLocalPlannerROS::localPlanCallback(const nav_msgs::Path::ConstPtr& msg)
{
    for(const auto& pose : msg->poses)
    {
        // Process local plan poses (poses from /move_base/TebLocalPlannerROS/local_plan)
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;
        
        //ROS_INFO("localPlanCallback :  x : %f y : %f z : %f ", x, y, z);
    }
}

bool TebLocalPlannerROS::getPath(std::vector<geometry_msgs::PoseStamped>& get_path) 
{
    get_path.clear();

    nav_msgs::Path teb_path;
    planner_->getPath(teb_path);  // 가상 함수 호출을 통해 teb_path를 채웁니다.

    for (const auto& pose : teb_path.poses)
    {
        geometry_msgs::PoseStamped pose_copy = pose;
        pose_copy.header.frame_id = "map";
        get_path.push_back(pose_copy);
    }

    return true;
}


/********************************************* Algorithm Function ***************************************************************/
void TebLocalPlannerROS::adjustPIDConstants(double & distance, double & kp, double & ki, double & kd) {

    // 거리가 가까워질수록 PID 상수를 낮춰 부드러운 감속을 유도

    if (distance < 0.7) {
        kp = 0.2;  // 적절한 값으로 조정
        ki = 0.05; // 적절한 값으로 조정
        kd = 0.1;  // 적절한 값으로 조정
    }
    else {
        kp = 0.5; // 기본 상수
        ki = 0.1; // 기본 상수
        kd = 0.3; // 기본 상수
    }
}
double TebLocalPlannerROS::SmoothDecelerationPIDControl(double current_velocity, double target_angle, double current_angle,double distance_to_target)
{

    double target_value = target_angle;
    double current_value = current_angle;
    double distance = distance_to_target;

    double kp = 0.5; // 비례 상수
    double ki = 0.1; // 적분 상수
    double kd = 0.3; // 미분 상수

    double target_linear_vel = 1.0; // 목표 선속도
    double current_linear_vel = current_velocity;

    // PID 제어 상수 설정

    double dt = 0.1; // 제어 주기

    double max_output = 1.0; // 최대 출력값
    double min_output = -1.0; // 최소 출력값
    double max_integrator = 10.0; // 적분값 최대값
    double min_integrator = -10.0; // 적분값 최소값

     adjustPIDConstants(distance, kp, ki, kd);

    static double error_sum = 0;
    static double last_error = 0;

    // 에러 계산
    double error = target_value - current_value;

    // 비례 제어
    double p_term = kp * error;

    // 적분 제어
    error_sum += error * dt;
    double i_term = ki * error_sum;

    // 미분 제어
    double derivative = (error - last_error) / dt;
    double d_term = kd * derivative;

    // PID 제어량 계산
    double output = p_term + i_term + d_term;

    // 출력 범위 제한
    output = std::max(std::min(output, max_output), min_output);

    // 적분 텀 제한
    error_sum = std::max(std::min(error_sum, max_integrator), min_integrator);

    // 마지막 에러 업데이트
    last_error = error;

    return output;
}


// 회전 속도 보정
double TebLocalPlannerROS::adjustAngularVelocity(double max_vel_theta,double yaw_diff, double reduction_threshold, double min_vel_scale)
{  
  double factor = fabs(yaw_diff) / M_PI;

  // 목표각도게 가까울수록 점진적인 감속 
  if(factor < reduction_threshold)
  {
    factor = reduction_threshold + (factor / reduction_threshold) * (1.0 - reduction_threshold);
  }

  double min_vel_theta = max_vel_theta * min_vel_scale;
  double angular_velocity = max_vel_theta * factor;
  return std::max(min_vel_theta, angular_velocity); // 최소 속도로 보정
}

//전방주시 거리점 가저오기, lookahead_dist와 transformed_plan을 기반으로 전방 주시 거리 계산 
geometry_msgs::PoseStamped TebLocalPlannerROS::getForwardViewDistance(
  const double& lookahead_dist, const std::vector<geometry_msgs::PoseStamped>& transformed_plan)
{
  // if (transformed_plan.empty()) {
  //   ROS_ERROR("getLookAheadPoint: The transformed_plan is empty!");
  //   return geometry_msgs::PoseStamped(); // return an empty PoseStamped to indicate an error
  // }

  // Find the first pose which is at a distance greater than the lookahead distance
  auto goal_pose_it = std::find_if(
    transformed_plan.begin(), transformed_plan.end(), [&](const geometry_msgs::PoseStamped& ps)
  {
    return std::hypot(ps.pose.position.x, ps.pose.position.y) >= lookahead_dist;
  });

  // If the number of poses is not far enough, take the last pose
  if (goal_pose_it == transformed_plan.end()) {
    if (transformed_plan.empty()) {
      //ROS_ERROR("getLookAheadPoint: The transformed_plan has no valid poses!");
      return geometry_msgs::PoseStamped(); // return an empty PoseStamped to indicate an error
    }
    //ROS_WARN("getLookAheadPoint: The lookahead distance is too large, using the last pose.");
    goal_pose_it = std::prev(transformed_plan.end());
  }

  geometry_msgs::PoseStamped goal_pose = *goal_pose_it;

  // frame_id가 비어 있는지 확인하고 설정
  if (goal_pose.header.frame_id.empty()) {
    //ROS_WARN("LookAheadPoint has an empty frame_id. Setting it to 'map'.");
    goal_pose.header.frame_id = "map"; // Default to 'map' if empty
  } else {
    ROS_INFO("LookAheadPoint frame_id: %s", goal_pose.header.frame_id.c_str());
  }

  return goal_pose;
}

// 목표점에 도달하여 제자리 회전할 것인지 판단
bool TebLocalPlannerROS::rotateAtTarget(const geometry_msgs::PoseStamped &robot_pose_geo)
{
  double dist_to_goal =  std::hypot(robot_pose_geo.pose.position.x, robot_pose_geo.pose.position.y);

  //ROS_WARN("dist_to_goal: %f, xy_tolerance: %f\n", dist_to_goal, cfg_.goal_tolerance.xy_goal_tolerance);
 
  if(dist_to_goal < cfg_.goal_tolerance.xy_goal_tolerance)  
  {
    //ROS_INFO("dist_to_goal: %f, xy_tolerance: %f\n ---\n", dist_to_goal, cfg_.goal_tolerance.xy_goal_tolerance);
    cfg_.goal_tolerance.xy_goal_tolerance = goal_dist_tol_temp_ + 0.2;
    return true;
  }
  return false;
}

// 탐색 도중 정지하여 방향 조절이 필요한지 판단
bool TebLocalPlannerROS::rotateForDetection(const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path)
{

  geometry_msgs::PoseStamped pose_in, pose_goal;
  pose_in = global_plan_.back();//teb_path.poses.back();
  pose_in.header.frame_id = "map";
  pose_in.header.stamp = ros::Time();
  tf_->transform(pose_in, pose_goal, "base_link");                  // 입력 포즈, 변환된 포즈 저장할 변수, 변환할 대상 프레임
  angle_to_path =  tf2::getYaw(pose_goal.pose.orientation);
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  //angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  //  ROS_INFO("shoudRotateToPath angle_to_path : %f", angle_to_path);

  return (fabs(angle_to_path) > cfg_.goal_tolerance.yaw_goal_tolerance);  
}

// 목표점 접근 시 정지 및 방향 조정 필요 여부 판단
bool TebLocalPlannerROS::rotateAndStopAtNearTarget(const geometry_msgs::PoseStamped & carrot_pose, double & angle_to_path)
{
  double dist_to_goal = std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y);
  geometry_msgs::PoseStamped pose_in, pose_goal;
  pose_in = global_plan_.back();//teb_path.poses.back();
  pose_in.header.frame_id = "map";
  pose_in.header.stamp = ros::Time();
  tf_->transform(pose_in, pose_goal, "base_link");                  // 입력 포즈, 변환된 포즈 저장할 변수, 변환할 대상 프레임
  angle_to_path =  tf2::getYaw(pose_goal.pose.orientation);

  //angle_to_path = std::atan2(carrot_pose.pose.position.y, carrot_pose.pose.position.x);
  //ROS_INFO("rotateAndStopAtNearTarget angle_to_path : %f", angle_to_path);

  if(fabs(angle_to_path) > goal_near_rotate_angular_)
  {
    goal_near_rotate_angular_ = 0.2;
    return true;
  }
  else
  {
    goal_near_rotate_angular_ = 0.4;
  }
  return false;
}

// 목표점 도달감속처리 
double TebLocalPlannerROS::approachVelocityScalingFactor(
  const nav_msgs::Path & transformed_path
) const
{
  // Waiting to apply the threshold based on integrated distance ensures we don't
  // erroneously apply approach scaling on curvy paths that are contained in a large local costmap.
  double remaining_distance = nav2_util::geometry_utils::calculate_path_length(transformed_path);
  if (remaining_distance < 0.8) {
    auto & last = transformed_path.poses.back();
    // Here we will use a regular euclidean distance from the robot frame (origin)
    // to get smooth scaling, regardless of path density.
    double distance_to_last_pose = std::hypot(last.pose.position.x, last.pose.position.y);
    return distance_to_last_pose / 0.8;
  } else {
    return 1.0;
  }
}

// 목표점에 다달기 감속처리
void TebLocalPlannerROS::applyApproachVelocityScaling(const nav_msgs::Path & path, double & linear_vel) const
{
  double approach_vel = linear_vel;
  double velocity_scaling = approachVelocityScalingFactor(path);
  double unbounded_vel = approach_vel * velocity_scaling;
  if (unbounded_vel < 0.05) {
    approach_vel = 0.05;
  } else {
    approach_vel *= velocity_scaling;
  }

  // Use the lowest velocity between approach and other constraints, if all overlapping
  linear_vel = std::min(linear_vel, approach_vel);
}

// 곡률감속처리
void TebLocalPlannerROS::applyConstraints(const double & curvature,const nav_msgs::Path & path, double & linear_vel, double sign)
{
  double curvature_vel = linear_vel;
  double cost_vel = linear_vel;

  // limit the linear velocity by curvature
  const double radius = fabs(1.0 / curvature);
  const double min_rad = 0.8;

  if (radius < min_rad) {
    curvature_vel *= 1.0 - (fabs(radius - min_rad) / min_rad);
  }
  
  // Use the lowest of the 2 constraint heuristics, but above the minimum translational speed
  linear_vel = std::min(cost_vel, curvature_vel);
  linear_vel = std::max(linear_vel, 0.2);

  // 경로 길이 계산, 설정 거리 도달 후 감속 처리
  applyApproachVelocityScaling(path, linear_vel);

  // Limit linear velocities to be valid
  linear_vel = clamp(fabs(linear_vel), 0.0, cfg_.robot.max_vel_x);
  linear_vel = sign * linear_vel;
}

// 안전검사 전방 측면 검사 장애물 감지시 거리에 따라 footprint_cost : -1 ~ -3 
bool TebLocalPlannerROS::inCollision(const double & x,  const double & y,  const double & theta)
{
//  costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses)

  double footprint_cost = costmap_model_.get()->footprintCost( x, y, theta, footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  //ROS_INFO( " foorprint_cost : %f ", footprint_cost );
  return footprint_cost < 0;
}

// 안전검사 맵좌표변환 적용
bool TebLocalPlannerROS::isCollisionImminent(const geometry_msgs::PoseStamped & robot_pose,const double & linear_vel, const double & angular_vel)
{
  // check current point is OK
  if (inCollision(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    tf::getYaw(robot_pose.pose.orientation)))
  {
    return true;
  }

  // visualization messages
  nav_msgs::Path arc_pts_msg;
  arc_pts_msg.header.frame_id = "map";
  arc_pts_msg.header.stamp = robot_pose.header.stamp;
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
  pose_msg.header.stamp = arc_pts_msg.header.stamp;


  geometry_msgs::Pose2D curr_pose;
  for(int i = 1; i < 8; i ++) {

    curr_pose.x = robot_pose.pose.position.x + (0.05 * i) * linear_vel * cos(curr_pose.theta);
    curr_pose.y = robot_pose.pose.position.y + (0.05 * i) * linear_vel * sin(curr_pose.theta);
    curr_pose.theta = tf::getYaw(robot_pose.pose.orientation); //+ (0.05 * i) * angular_vel;

    // store it for visualization
    pose_msg.pose.position.x = curr_pose.x;
    pose_msg.pose.position.y = curr_pose.y;
    pose_msg.pose.position.z = 0.01;
    arc_pts_msg.poses.push_back(pose_msg);

    // check for collision at the projected pose
    if (inCollision(curr_pose.x, curr_pose.y, curr_pose.theta)) {
//      carrot_arc_pub_.publish(arc_pts_msg);
      return true;
    }
  }

//  carrot_arc_pub_.publish(arc_pts_msg);
  return false;
}

// 뒤로 갈 지 여부
bool TebLocalPlannerROS::shouldMoveBackward( const std::vector<geometry_msgs::PoseStamped>& transformed_plan_local)
{
  static bool begin_done = false;

  if(begin_done) {

    // range 리턴 
    for (const auto& pose : transformed_plan_local) {
      if( pose.pose.position.x < -0.005 ) {
        begin_done = true;
        return true;
      } 
    }
    begin_done = false;
    return false;
  }

  for (const auto& pose : transformed_plan_local) {

    if( (pose.pose.position.x < -0.04 && transformed_plan_local.back().pose.position.x > 0.1) || begin_done ) {
      begin_done = true;

      geometry_msgs::PoseStamped pose_in, pose_back;
      pose_in.header.frame_id = "base_link";
      pose_in.header.stamp = ros::Time(0);
      pose_in.pose.position.x = -0.1;
      pose_in.pose.position.y = 0.0;
      pose_in.pose.position.z = 0.0;
      pose_in.pose.orientation = pose.pose.orientation; 

      tf_->transform(pose_in, pose_back, "map");
      pose_back.header.frame_id = "map";

//      double footprint_cost = costmap_model_->footprintCost(pose_back.pose.position.x, pose_back.pose.position.y,
//        tf2::getYaw(pose_back.pose.orientation), costmap_ros_->getRobotFootprint());

      if( ! (pose_back.pose.position.x, pose_back.pose.position.y, tf::getYaw(pose_back.pose.orientation)) )
      {
        return true;
      } 
        
    }

  }

  return false;
}

//  전방, 측면 10~20cm 장애물 여부 측정
bool TebLocalPlannerROS::frontCheckObs(const geometry_msgs::PoseStamped robot_pose)
{
    nav_msgs::Path arc_pts_msg;
    arc_pts_msg.header.frame_id = "map";
    arc_pts_msg.header.stamp = robot_pose.header.stamp;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = arc_pts_msg.header.frame_id;
    pose_msg.header.stamp = arc_pts_msg.header.stamp;

    std::vector<geometry_msgs::PoseStamped> front_path;

    geometry_msgs::PoseStamped pose_in;
    pose_in.header.frame_id = "base_link";
    pose_in.header.stamp = ros::Time(0);
    pose_in.pose.position.y = 0.0;
    pose_in.pose.position.z = 0.0;
    pose_in.pose.orientation.w = 1; // robot_pose.pose.orientation; 

    for (int i = 0; i < 30; i++)
    {
        pose_in.pose.position.x = i * 0.025;
        front_path.push_back(pose_in);
    }

    for (int i = 0; i < 5; i++)
    {
        pose_in.pose.position.y = -i * 0.025;
        pose_in.pose.position.x = -pose_in.pose.position.y;
        front_path.push_back(pose_in);
    }

    for (int i = 0; i < 5; i++)
    {
        pose_in.pose.position.y = i * 0.025;
        pose_in.pose.position.x = pose_in.pose.position.y;
        front_path.push_back(pose_in);
    }

    for (const auto& pose : front_path)
    {
        geometry_msgs::PoseStamped global_pose;

        // Check frame_id before transformation
        if (pose.header.frame_id.empty())
        {
            //ROS_WARN("Skipping transformation as input pose has an empty frame_id.");
            //continue;
        }

        try
        {
            tf_->transform(pose, global_pose, "map");
            global_pose.header.frame_id = "map";
            arc_pts_msg.poses.push_back(global_pose);

            if (inCollision(global_pose.pose.position.x, global_pose.pose.position.y, tf::getYaw(global_pose.pose.orientation)))
            {
                return true;
            }
        }
        catch (tf2::TransformException& ex)
        {
            ROS_ERROR("Failed to transform pose from %s to map: %s", pose.header.frame_id.c_str(), ex.what());
        }
    }

    carrot_arc_pub_.publish(arc_pts_msg);

    return false;
}

/* -------------------------------------- Library Start ----------------------------------   */

bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t TebLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                     const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped &cmd_vel,
                                                     std::string &message)
{
  // check if plugin initialized
  if(!initialized_)
  {
    ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
    message = "teb_local_planner has not been initialized";
    return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }

  static uint32_t seq = 0;
  cmd_vel.header.seq = seq++;
  cmd_vel.header.stamp = ros::Time::now();
  cmd_vel.header.frame_id = robot_base_frame_;
  cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
  goal_reached_ = false;  
  
  // Get robot pose
  geometry_msgs::PoseStamped robot_pose;
  costmap_ros_->getRobotPose(robot_pose);
  robot_pose_ = PoseSE2(robot_pose.pose);
    
  // Get robot velocity
  geometry_msgs::PoseStamped robot_vel_tf;
  odom_helper_.getRobotVel(robot_vel_tf);
  robot_vel_.linear.x = robot_vel_tf.pose.position.x;
  robot_vel_.linear.y = robot_vel_tf.pose.position.y;
  robot_vel_.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);
  
  // prune global plan to cut off parts of the past (spatially before the robot)
  pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

  // Transform global plan to the frame of interest (w.r.t. the local costmap)
  std::vector<geometry_msgs::PoseStamped> transformed_plan;
  int goal_idx;
  geometry_msgs::TransformStamped tf_plan_to_global;
  if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist, 
                           transformed_plan, &goal_idx, &tf_plan_to_global))
  {
    //ROS_WARN("Could not transform the global plan to the frame of the controller");
    message = "Could not transform the global plan to the frame of the controller";
    return mbf_msgs::ExePathResult::INTERNAL_ERROR;
  }

  // update via-points container
  if (!custom_via_points_active_)
    updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);

  // check if global goal is reached
  geometry_msgs::PoseStamped global_goal;
  tf2::doTransform(global_plan_.back(), global_goal, tf_plan_to_global);
  double dx = global_goal.pose.position.x - robot_pose_.x();
  double dy = global_goal.pose.position.y - robot_pose_.y();


  double delta_orient = g2o::normalize_theta( tf2::getYaw(global_goal.pose.orientation) - robot_pose_.theta() );
  if(fabs(std::sqrt(dx*dx+dy*dy)) < cfg_.goal_tolerance.xy_goal_tolerance
    && fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance
    && (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
    && (base_local_planner::stopped(base_odom, cfg_.goal_tolerance.theta_stopped_vel, cfg_.goal_tolerance.trans_stopped_vel)
        || cfg_.goal_tolerance.free_goal_vel))
  {
    goal_reached_ = true;
    return mbf_msgs::ExePathResult::SUCCESS;
  }

  // check if we should enter any backup mode and apply settings
  configureBackupModes(transformed_plan, goal_idx);
  
    
  // Return false if the transformed global plan is empty
  if (transformed_plan.empty())
  {
    //ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
    message = "Transformed plan is empty";
    return mbf_msgs::ExePathResult::INVALID_PATH;
  }
              
  // Get current goal point (last point of the transformed plan)
  robot_goal_.x() = transformed_plan.back().pose.position.x;
  robot_goal_.y() = transformed_plan.back().pose.position.y;
  
  // tf::Stamped<tf::Pose> goal_point;
  // tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
  // robot_goal_.x() = goal_point.getOrigin().getX();
  // robot_goal_.y() = goal_point.getOrigin().getY();

  // Overwrite goal orientation if needed
  if (cfg_.trajectory.global_plan_overwrite_orientation)
  {
    robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, transformed_plan.back(), goal_idx, tf_plan_to_global);
    // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
    tf2::Quaternion q;
    q.setRPY(0, 0, robot_goal_.theta());
    tf2::convert(q, transformed_plan.back().pose.orientation);
  }  
  else
  {
    robot_goal_.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
  }

  // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
  if (transformed_plan.size()==1) // plan only contains the goal
  {
    transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
  }
  transformed_plan.front() = robot_pose; // update start
    
  // clear currently existing obstacles
  obstacles_.clear();
  
  // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
  if (costmap_converter_)
    updateObstacleContainerWithCostmapConverter();
  else
    updateObstacleContainerWithCostmap();
  
  // also consider custom obstacles (must be called after other updates, since the container is not cleared)
  updateObstacleContainerWithCustomObstacles();
  
    
  // Do not allow config changes during the following optimization step
  boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

  // if(fabs(std::sqrt(dx*dx+dy*dy)) > cfg_.goal_tolerance.xy_goal_tolerance)
  // {
  //   transformed_plan[transformed_plan.size() - 1].pose.orientation = transformed_plan[0].pose.orientation;  
  // }  
  // Now perform the actual planning
//   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
  bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
  if (!success)
  {
    planner_->clearPlanner(); // force reinitialization for next time
    //ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner was not able to obtain a local plan";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Check for divergence
  // if (planner_->hasDiverged())
  // {
  //   cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

  //   // Reset everything to start again with the initialization of new trajectories.
  //   planner_->clearPlanner();
  //   ROS_WARN_THROTTLE(1.0, "TebLocalPlannerROS: the trajectory has diverged. Resetting planner...");

  //   ++no_infeasible_plans_; // increase number of infeasible solutions in a row
  //   time_last_infeasible_plan_ = ros::Time::now();
  //   last_cmd_ = cmd_vel.twist;
  //   return mbf_msgs::ExePathResult::NO_VALID_CMD;
  // }
         
  // Check feasibility (but within the first few states only)
  if(cfg_.robot.is_footprint_dynamic)
  {
    // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
    footprint_spec_ = costmap_ros_->getRobotFootprint();
    costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
  }

  bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses, cfg_.trajectory.feasibility_check_lookahead_distance);
  if (!feasible)
  {
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

    // now we reset everything to start again with the initialization of new trajectories.
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");
    
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner trajectory is not feasible";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }

  // Get the velocity command for this sampling interval
  if (!planner_->getVelocityCommand(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.trajectory.control_look_ahead_poses))

  {
    planner_->clearPlanner();
    ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
    ++no_infeasible_plans_; // increase number of infeasible solutions in a row
    time_last_infeasible_plan_ = ros::Time::now();
    last_cmd_ = cmd_vel.twist;
    message = "teb_local_planner velocity command invalid";
    return mbf_msgs::ExePathResult::NO_VALID_CMD;
  }
  
  // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints). // FW - 01 제외 
  saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z,
                   cfg_.robot.max_vel_x, cfg_.robot.max_vel_y, cfg_.robot.max_vel_trans, cfg_.robot.max_vel_theta, 
                   cfg_.robot.max_vel_x_backwards);

  // convert rot-vel to steering angle if desired (carlike robot).
  // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
  // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
  if (cfg_.robot.cmd_angle_instead_rotvel)
  {
    cmd_vel.twist.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z,
                                                                cfg_.robot.wheelbase, 0.95*cfg_.robot.min_turning_radius);
    if (!std::isfinite(cmd_vel.twist.angular.z))
    {
      cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;
      last_cmd_ = cmd_vel.twist;
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      message = "teb_local_planner steering angle is not finite";
      return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }
  }

  /* -----------------------------------------  Tfoi Function start -------------------------------------- */
  
cmd_vel.twist.linear.x = cfg_.robot.max_vel_x;

// teb path log test 
#if ROS_DEBUG_TEST_01
for(const auto& path_pose : teb_path.poses)
    {
        ROS_INFO("Pose: x=%f, y=%f, z=%f", 
            path_pose.pose.position.x,
            path_pose.pose.position.y,
            path_pose.pose.position.z);
        
        ROS_INFO("Orientation: x=%f, y=%f, z=%f, w=%f", 
            path_pose.pose.orientation.x,
            path_pose.pose.orientation.y,
            path_pose.pose.orientation.z,
            path_pose.pose.orientation.w);
    }
#endif

//계획된 경로 가져오기 
//nav_msgs::Path teb_path;
//planner_->getPath(teb_path);

// teb_path의 상태를 로깅
#if ROS_DEBUG_TEST_02 
if (teb_path.poses.empty()) {
    ROS_ERROR("teb_path is empty. No poses available for transformation.");
} else {
    ROS_INFO("teb_path contains %zu poses.", teb_path.poses.size());
}
#endif 

// 변환된 경로의 전역 좌표를 base_link 좌표로 변환

std::vector<geometry_msgs::PoseStamped> transformed_local_plan;

  for (const auto& pose : teb_path.poses) 
  {
    try
    {
      // 변환된 결과를 저장하는 PoseStamped 메시지 만들기

      geometry_msgs::PoseStamped transformed_point;

      // 입력점의 헤더 정보 설정
      transformed_point.header = pose.header;
      // 프레임 ID 확인
      if (pose.header.frame_id.empty())
        {
            //ROS_WARN("Input pose has an empty frame_id. Skipping this pose.");
            //continue; // frame_id가 비어 있으면 변환 시도하지 않음
        }

     // 좌표 변환을 진행하여 입력점을 map 좌표계에서 base_link 좌표계로 변환
        tf_->transform(pose, transformed_point, "base_link"); // 입력 포즈, 변환된 포즈 저장할 변수, 변환할 대상 프레임

      transformed_local_plan.push_back(transformed_point);

    }
    catch(tf2::TransformException& ex)
    {
      #if ROS_DEBUG_ERROR_01 
      ROS_ERROR("Failed to transform path: %s", ex.what());
      #endif
    }
  }

#if ROS_DEBUG_TEST_03
// transformed_local_plan 길이를 체크하여 로깅
if (transformed_local_plan.empty()) {
    ROS_ERROR("Transformed local plan is empty after attempting to transform poses.");
} else {
    ROS_INFO("Transformed local plan contains %zu poses.", transformed_local_plan.size());
}
#endif
  
  nav_msgs::Path local_path;
  local_path.header.stamp = ros::Time::now();
  local_path.header.frame_id = "base_link";
  for (const auto& pose : transformed_local_plan) {

    local_path.poses.push_back(pose);
  }

  // 경로가 교차하는지 판단후 리셋
  geometry_msgs::PoseStamped pose_in, pose_goal;
  pose_in = global_plan_.back();//teb_path.poses.back();
  pose_in.header.frame_id = "map";
  pose_in.header.stamp = ros::Time();
  tf_->transform(pose_in, pose_goal, "base_link");                  // 입력 포즈, 변환된 포즈 저장할 변수, 변환할 대상 프레임

  std::vector<geometry_msgs::PoseStamped> teb_poses = teb_path.poses;
  uint8_t teb_num = 0;
  
  //ROS_INFO(" pose_goal YAW : %f", tf2::getYaw(pose_goal.pose.orientation));
  //ROS_INFO( "pose to distance : %f", std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y) );

  for(int i = 1; i < teb_poses.size(); i++)
  {
    double delta_x = teb_poses[i].pose.position.x - teb_poses[i-1].pose.position.x; 
    double delta_y = teb_poses[i].pose.position.y - teb_poses[i-1].pose.position.y;
    double distance = sqrt(delta_x*delta_x + delta_y*delta_y);  

    if(distance > 0.5) 
    {
    #if ROS_DEBUG_TEST_04      
      ROS_WARN("two point distance > 0.5 Reset Teb Planner!!!"); 
    #endif
  //  teb_num = 15;
  //  break;
    }
    //ROS_INFO(" for pose_goal distance : %f", distance); 
    geometry_msgs::PoseStamped pose = teb_poses[i];
    // Convert quaternion to Euler angles
    tf::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    #if ROS_DEBUG_TEST_10
    ROS_INFO("roll pithch yaw : %f, %f, %f", roll, pitch, yaw);
    #endif
    // Check if yaw angle is within specified ranges
    if(yaw >= 0 && yaw <= M_PI/2)
    {
      teb_num |= 0x01;
    }
    else if(yaw > M_PI/2 && yaw <= M_PI)
    {
      teb_num |= 0x02;
    }
    else if(yaw >= -M_PI && yaw < -M_PI/2)
    {
      teb_num |= 0x04;
    }
    else if(yaw >= -M_PI/2 && yaw < 0)
    {
      teb_num |= 0x08;
    }
  }

  //   if(teb_num == 15 /*&& fabs(std::sqrt(dx*dx+dy*dy) > 2)*/ && pose_goal.pose.position.x > 0.01)
  // {
  //   planner_->clearPlanner(); 
  //   cmd_vel.twist = last_cmd_;

  //   ROS_WARN("Reset Teb Planner!!!");
  //   return true;
  // }  

  //목표점 업데이트 여부 판단
  static double last_goal_x = 0.0;
  static double last_goal_y = 0.0;

  if( last_goal_x != pose_in.pose.position.x || last_goal_y != pose_in.pose.position.y )
  {
    need_rotate_ = true;
    //ROS_INFO("need_rotate_ : %s", need_rotate_ ? "true" : "false");

  }

  last_goal_x = pose_in.pose.position.x;
  last_goal_y = pose_in.pose.position.y;

  // 전방시 거리점 계산

  double lookahead_dist = 0.7;

  tf::Stamped<tf::Pose> tf_pose; // tf::Pose를 저장할 변수
  geometry_msgs::PoseStamped robot_pose_geo; // 변환된 geometry_msgs::PoseStamped를 저장할 변수

  tf::poseStampedMsgToTF(robot_pose, tf_pose); // geometry_msgs::PoseStamped를 tf::Stamped<tf::Pose>로 변환
  tf::poseTFToMsg(tf_pose, robot_pose_geo.pose); // tf::Pose를 geometry_msgs::Pose로 변환
  //ROS_INFO("robot_pose_geo.pose - x: %f, y: %f, z: %f", robot_pose_geo.pose.position.x, robot_pose_geo.pose.position.y, robot_pose_geo.pose.position.z);
 
  if( frontCheckObs(robot_pose_geo) )
  {
    lookahead_dist = 0.1;
    //ROS_INFO("lookahead_dist   :::   %f",lookahead_dist);
  } 
// computeVelocityCommands 함수 내에서
geometry_msgs::PoseStamped carrot_pose = getForwardViewDistance(lookahead_dist, transformed_local_plan);
//ROS_INFO(" getForwadrview distance : %f ", std::hypot(carrot_pose.pose.position.x, carrot_pose.pose.position.y));
//if (carrot_pose.header.frame_id.empty()) {
//ROS_ERROR("Failed to get a valid lookahead point. Skipping velocity computation.");
//If the carrot_pose is invalid, handle the error appropriately here
//... maybe set some safe default or return an even indicating an error.
//return mbf_msgs::ExePathResult::NO_VALID_CMD;
//}

  geometry_msgs::PointStamped carrot_msg;
  carrot_msg.header =  robot_pose_geo.header; //carrot_pose.header;
  carrot_msg.point.x = robot_pose_geo.pose.position.x; // carrot_pose.pose.position.x;
  carrot_msg.point.y = robot_pose_geo.pose.position.y; //carrot_pose.pose.position.y;
  carrot_msg.point.z = 0.01;  // publish right over map to stand out
  carrot_pub_.publish(carrot_msg);


  // 곡률 계산
  const double carrot_dist2 =
    //(robot_pose_geo.pose.position.x * robot_pose_geo.pose.position.x) +
    //(robot_pose_geo.pose.position.y * robot_pose_geo.pose.position.y);
     (carrot_pose.pose.position.x * carrot_pose.pose.position.x) +
    (carrot_pose.pose.position.y * carrot_pose.pose.position.y);
  //ROS_INFO("carrot_dist2 : %f ", carrot_dist2);
  // Find curvature of circle (k = 1 / R)
  double curvature = 0.0;
  if (carrot_dist2 > 0.001) 
  {
    curvature = 2.0 * carrot_pose.pose.position.y  / carrot_dist2;  
  }

  double angle_to_heading;  

  double cur_yaw = robot_pose_.theta();
  double global_plan_0_yaw = tf::getYaw(global_plan_[0].pose.orientation); 

  //ROS_INFO("global_plan - cur_yaw  %f   %f   %f ",cur_yaw,global_plan_0_yaw,std::min( fabs(global_plan_0_yaw - cur_yaw),2.0 * M_PI - fabs(global_plan_0_yaw - cur_yaw)));

  double dist_to_goal_final = std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y);

// test_start 


// test_end 
  // 뒤로 갈것인지 판단 

  // bool frontcheckobstest = frontCheckObs(robot_pose_geo);
  // bool shouldMoveBackwardtest =  shouldMoveBackward(transformed_local_plan);
  // std::cout << "frontcheckobs : " << frontcheckobstest;
  // std::cout << "shouldMoveBackward :" << shouldMoveBackwardtest;
  
  double sign; 
  if( shouldMoveBackward(transformed_local_plan) && frontCheckObs(robot_pose_geo) ) {
    ROS_WARN("check if need go back");
    cmd_vel.twist.linear.x = -0.1;
    cmd_vel.twist.angular.z = 0;
  }  
  else if (rotateAtTarget(robot_pose_geo)) { // //  로봇과 전방시점 거리가 설정 거리보다 작아 제자리에서 회전하며 방향 조정
    double angle_to_goal = tf::getYaw(pose_goal.pose.orientation);
    //rotateToHeading(cmd_vel.twist.linear.x, cmd_vel.twist.angular.z, angle_to_heading);
    //double sign = angle_to_goal > 0.0 ? 1.0 : -1.0;
    if (angle_to_goal > 0.0)
      sign = 1.0;
    else 
      sign = -1.0;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = sign * 0.35;
    if(angle_to_goal < cfg_.goal_tolerance.yaw_goal_tolerance){
      cmd_vel.twist.angular.z = 0;
    }
  }   
  else if ( (rotateForDetection( robot_pose_geo , angle_to_heading) && frontCheckObs(robot_pose_geo) ) ||     //로봇 현재좌표와 전방시거리점의 tan각도가 설정각도보다 크면 정지하여 회전시켜 각도를 조정한다 로봇 현재위치와 전방시점 협각도가 0.2보다 크고 전방에 장애물 

  ( dist_to_goal_final < 1.5 && rotateAndStopAtNearTarget( robot_pose_geo, angle_to_heading) ) ) {  // carrot_pose
    double angle_to_goal = tf::getYaw(pose_goal.pose.orientation);
        if (angle_to_goal > 0.0)
      sign = 1.0;
    else 
      sign = -1.0;    
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = sign * 0.35;  
  }
 //시작점 회전
  else if( std::min( fabs(global_plan_0_yaw - cur_yaw),2.0 * M_PI - fabs(global_plan_0_yaw - cur_yaw)) > 0.4 && need_rotate_) {
    //double angle_to_path = std::atan2(robot_pose_geo.pose.position.y, robot_pose_geo.pose.position.x);   
    double angle_to_goal = tf::getYaw(pose_goal.pose.orientation);
    if (angle_to_goal > 0.0)
      sign = 1.0;
    else 
      sign = -1.0;
    
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.angular.z = sign * 0.35; 
    //}
  }            
  else { //곡률감속처리
    //applyConstraints(curvature, local_path, cmd_vel.twist.linear.x, 1.0);

    // 각속도설정
    // double yt = carrot_pose.pose.position.y;
    // double ld_2 = lookahead_dist * lookahead_dist;
    // double dist_to_goal_final = std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y);
    // cmd_vel.twist.angular.z = 1.2 * cmd_vel.twist.linear.x / ld_2 * yt;
    // //Ensure that angular_vel does not exceed user-defined amount
    // cmd_vel.twist.angular.z = clamp(cmd_vel.twist.angular.z, -cfg_.robot.max_vel_theta, cfg_.robot.max_vel_theta);
    // if(dist_to_goal_final <= 1)//dxscpp  로봇이 목표점에서 1m보다 작을 경우 각속도는 종전의 1.5배
    // {
    //   //ROS_INFO("dist_to_goal_final is: %f\n", dist_to_goal_final);
    //   cmd_vel.twist.angular.z *= 1.5;
    // }
    need_rotate_ = false;    
  }

  if(fabs(chiss_angular_) > 17)  //   바퀴 0으로 셋팅
  {
    if(fabs(chiss_linear_) < 0.05 && fabs(cmd_vel.twist.linear.x) >= 0.05)
    {
      cmd_vel.twist.linear.x = 0;
      cmd_vel.twist.angular.z = 0;
//      ROS_WARN("Wait reset!");
    }    
  }

   // 충돌 감지 및 정지
  geometry_msgs::PoseStamped current_robot_pose;
  bool got_robot_pose = costmap_ros_->getRobotPose(current_robot_pose);

  // 같은 값 나오는 지 확인 
  #if ROS_DEBUG_TEST_05
  ROS_INFO("robot_pose_geo x : %f  y : %f, yaw : %f", robot_pose_geo.pose.position.x, robot_pose_geo.pose.position.y , tf::getYaw(robot_pose_geo.pose.orientation));
  //ROS_WARN("current_robot_pose x : %f  y : %f, yaw : %f", current_robot_pose.pose.position.x, current_robot_pose.pose.position.y , tf::getYaw(current_robot_pose.pose.orientation));
  #endif 

  #if ROS_DEBUG_TEST_08
  ROS_INFO("angle to path %f", tf2::getYaw(pose_goal.pose.orientation));
  #endif

  #if ROS_DEBUG_TEST_09
  ROS_INFO("goal to distance %f", fabs(std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y)));
  #endif

  if ( isCollisionImminent(robot_pose_geo, cmd_vel.twist.linear.x, cmd_vel.twist.angular.z) ) { //current_robot_pose
    ROS_WARN("Detected collision ahead!");
    cmd_vel.twist.linear.x = 0;
    cmd_vel.twist.angular.z = 0;
  }
  

// 구간 시작
 #if ROS_DEBUG_TEST_11
if (start_time_flag == false)
{
    start_time_flag = true;
    start_time = ros::Time::now();  
     end_time = ros::Time::now();  
}
else
{
  end_time = ros::Time::now();  
}
#endif


#if 1

if(flag_distance == false)
{

  // 구간 끝
  if (fabs(std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y)) < cfg_.goal_tolerance.xy_goal_tolerance)
  {

    #if 1
      ROS_ERROR("//===================================================="); 
      ROS_ERROR("//[Setp #2] Tfoi was goal reached(Distance). (x , y)");
      ROS_ERROR("//===================================================="); 
    #endif 
  
    flag_distance = true;

  }
  else
  {
       #if 1
          ROS_ERROR("//===================================================="); 
          ROS_ERROR("//[Setp #1] Tolerance GOAL Distance");
          ROS_ERROR("//   - Robot to Distance for GOAL : %f ", fabs(std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y)));
          ROS_ERROR("//   - goal_tolerance_xy_goal     : %f ", cfg_.goal_tolerance.xy_goal_tolerance);
          ROS_ERROR("//   - goal_reached : %d ",  goal_reached_);
          ROS_ERROR("//===================================================="); 

      #endif

  }

}
else
{

    if (fabs(tf::getYaw(pose_goal.pose.orientation)) < cfg_.goal_tolerance.yaw_goal_tolerance)
    {
        #if ROS_DEBUG_TEST_07
        ROS_ERROR("//===================================================="); 
        ROS_ERROR("//[Setp #3-2]Tfoi was goal reached. (Yaw....!!!)");
        ROS_ERROR("//===================================================="); 
        #endif

        goal_reached_ = true;
        //flag_distance = false;
        
        return mbf_msgs::ExePathResult::SUCCESS;
    }
    else
    {      
        
          if ( (rotateForDetection( robot_pose_geo , angle_to_heading) && frontCheckObs(robot_pose_geo) ) ||     //로봇 현재좌표와 전방시거리점의 tan각도가 설정각도보다 크면 정지하여 회전시켜 각도를 조정한다 로봇 현재위치와 전방시점 협각도가 0.2보다 크고 전방에 장애물 
                 ( dist_to_goal_final < 1.5 && rotateAndStopAtNearTarget( robot_pose_geo, angle_to_heading) ) ) {  // carrot_pose
                    double angle_to_goal = tf::getYaw(pose_goal.pose.orientation);
                
                if (angle_to_goal > 0.0)
                {
                  sign = 1.0;
                }
                else
                {
                  sign = -1.0;    
                }
                 
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = sign * 0.35;  
                #if 1
                ROS_ERROR("=========================================== ");
                ROS_ERROR("//[Setp #3-3]Rotating .... ");
                ROS_ERROR("  - sign %f : ", sign);
                ROS_ERROR("=========================================== ");
                #endif 
            }          
    }

    #if 1
          ROS_ERROR("//===================================================="); 
          ROS_ERROR("// [Setp #3-1]Tolerance Tolerance GOAL Yaw NG Check #2");
          ROS_ERROR("//   - Robot to Angle for GOAL : %f ", fabs(tf::getYaw(pose_goal.pose.orientation)));
          ROS_ERROR("//   - goal_yaw_tolerance_goal : %f ", cfg_.goal_tolerance.yaw_goal_tolerance);          
          ROS_ERROR("//   - goal_reached : %d ",  goal_reached_);
          ROS_ERROR("//===================================================="); 
    #endif


}

#else

// 구간 끝
if (fabs(std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y)) < cfg_.goal_tolerance.xy_goal_tolerance)
{
    #if ROS_DEBUG_TEST_06
    ROS_ERROR("Tfoi was goal reached. (x , y)");
    #endif 
    
    if (fabs(tf::getYaw(pose_goal.pose.orientation)) < cfg_.goal_tolerance.yaw_goal_tolerance)
    {
        #if ROS_DEBUG_TEST_07
        ROS_ERROR("Tfoi was goal reached. (Yaw)");
        #endif
        goal_reached_ = true;
        return mbf_msgs::ExePathResult::SUCCESS;
    }
    else
    {
          #if ROS_DEBUG_ERROR_02
          ROS_ERROR("//===================================================="); 
          ROS_ERROR("// Tolerance GOAL Distance OK,Tolerance GOAL Yaw NG");
          ROS_ERROR("//   - Robot to Angle for GOAL : %f ", fabs(tf::getYaw(pose_goal.pose.orientation)));
          ROS_ERROR("//   - goal_yaw_tolerance_goal : %f ", cfg_.goal_tolerance.yaw_goal_tolerance);
          ROS_ERROR("//   - Robot to Distance for GOAL : %f ", fabs(std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y)));
          ROS_ERROR("//   - goal_tolerance_xy_goal     : %f ", cfg_.goal_tolerance.xy_goal_tolerance);
          ROS_ERROR("//===================================================="); 
          #endif
          if ( (rotateForDetection( robot_pose_geo , angle_to_heading) && frontCheckObs(robot_pose_geo) ) ||     //로봇 현재좌표와 전방시거리점의 tan각도가 설정각도보다 크면 정지하여 회전시켜 각도를 조정한다 로봇 현재위치와 전방시점 협각도가 0.2보다 크고 전방에 장애물 
                 ( dist_to_goal_final < 1.5 && rotateAndStopAtNearTarget( robot_pose_geo, angle_to_heading) ) ) {  // carrot_pose
                    double angle_to_goal = tf::getYaw(pose_goal.pose.orientation);
                
                if (angle_to_goal > 0.0)
                {
                  sign = 1.0;
                }
                else
                {
                  sign = -1.0;    
                }
                 
                cmd_vel.twist.linear.x = 0.0;
                cmd_vel.twist.angular.z = sign * 0.35;  
                ROS_ERROR("============== ");
                ROS_ERROR("//Ratating .... ");
                ROS_ERROR("  - sign %f : ", sign);
                ROS_ERROR("============== ");

            }          
    }
}
else
{   
      #if ROS_DEBUG_ERROR_03  
      ROS_ERROR("//===================================================="); 
      ROS_ERROR("// Tolerance GOAL Distance NG");
      ROS_ERROR("//   - Robot to Distance for GOAL : %f ", fabs(std::hypot(pose_goal.pose.position.x, pose_goal.pose.position.y)));
      ROS_ERROR("//   - goal_tolerance_xy_goal     : %f ", cfg_.goal_tolerance.xy_goal_tolerance);
      ROS_ERROR("//   - Robot to Angle for GOAL : %f ", fabs(tf::getYaw(pose_goal.pose.orientation)));
      ROS_ERROR("//   - goal_yaw_tolerance_goal : %f ", cfg_.goal_tolerance.yaw_goal_tolerance);
      ROS_ERROR("//====================================================");     
      #endif
}

#endif

    // 구간 종료 시간 설정
    //end_time = ros::Time::now();  
    // 구간 시간 측정
    #if ROS_DEBUG_TEST_11
    ros::Duration duration = end_time - start_time;
    double duration_ms = duration.toSec() * 1000; // 초를 밀리초로 변환
    ROS_INFO("*1 Duration : %.3f ms", duration_ms);
    ROS_INFO("distance_flag : %d",  flag_distance);
  
    if(duration_ms > 75.0)
    {    
        ROS_ERROR("//===================================================="); 
        ROS_ERROR("// ERROR Duration : %.3f ms", duration_ms);
        ROS_ERROR("//===================================================="); 
    }

    start_time = ros::Time::now(); 
    #endif



  /*----------------------------------------------- Visualize -------------------------------------------- */
  
  // a feasible solution should be found, reset counter
  no_infeasible_plans_ = 0;  

  // store last command (for recovery analysis etc.)
  last_cmd_ = cmd_vel.twist;
  
  // Now visualize everything    
  planner_->visualize();
  visualization_->publishObstacles(obstacles_, costmap_->getResolution());
  visualization_->publishViaPoints(via_points_);
  visualization_->publishGlobalPlan(global_plan_);
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool TebLocalPlannerROS::isGoalReached()
{
  if (goal_reached_)
  {
    ROS_INFO("GOAL Reached!");
    planner_->clearPlanner();
    return true;
  }
  return false;
}


void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{  
  // Add costmap obstacles if desired
  if (cfg_.obstacles.include_costmap_obstacles)
  {
    Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();
    
    for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)
    {
      for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)
      {
        if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)
        {
          Eigen::Vector2d obs;
          costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));
            
          // check if obstacle is interesting (e.g. not far behind the robot)
          Eigen::Vector2d obs_dir = obs-robot_pose_.position();
          if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )
            continue;
            
          obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
        }
      }
    }
  }
}

void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
  if (!costmap_converter_)
    return;
    
  //Get obstacles from costmap converter
  costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
  if (!obstacles)
    return;

  for (std::size_t i=0; i<obstacles->obstacles.size(); ++i)
  {
    const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
    const geometry_msgs::Polygon* polygon = &obstacle->polygon;

    if (polygon->points.size()==1 && obstacle->radius > 0) // Circle
    {
      obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
    }
    else if (polygon->points.size()==1) // Point
    {
      obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
    }
    else if (polygon->points.size()==2) // Line
    {
      obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                        polygon->points[1].x, polygon->points[1].y )));
    }
    else if (polygon->points.size()>2) // Real polygon
    {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (std::size_t j=0; j<polygon->points.size(); ++j)
        {
            polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
    }

    // Set velocity, if obstacle is moving
    if(!obstacles_.empty())
      obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
  }
}


void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
  // Add custom obstacles obtained via message
  boost::mutex::scoped_lock l(custom_obst_mutex_);

  if (!custom_obstacle_msg_.obstacles.empty())
  {
    // We only use the global header to specify the obstacle coordinate system instead of individual ones
    Eigen::Affine3d obstacle_to_map_eig;
    try 
    {
      geometry_msgs::TransformStamped obstacle_to_map =  tf_->lookupTransform(global_frame_, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                                                                              custom_obstacle_msg_.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));
      obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s",ex.what());
      obstacle_to_map_eig.setIdentity();
    }
    
    for (size_t i=0; i<custom_obstacle_msg_.obstacles.size(); ++i)
    {
      if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0 ) // circle
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new CircularObstacle( (obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 ) // point
      {
        Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                             custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        obstacles_.push_back(ObstaclePtr(new PointObstacle( (obstacle_to_map_eig * pos).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2 ) // line
      {
        Eigen::Vector3d line_start( custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                    custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z );
        Eigen::Vector3d line_end( custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                  custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z );
        obstacles_.push_back(ObstaclePtr(new LineObstacle( (obstacle_to_map_eig * line_start).head(2),
                                                           (obstacle_to_map_eig * line_end).head(2) )));
      }
      else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
      {
        //ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
        continue;
      }
      else // polygon
      {
        PolygonObstacle* polyobst = new PolygonObstacle;
        for (size_t j=0; j<custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
        {
          Eigen::Vector3d pos( custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                               custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z );
          polyobst->pushBackVertex( (obstacle_to_map_eig * pos).head(2) );
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if(!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
    }
  }
}

void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
  via_points_.clear();
  
  if (min_separation<=0)
    return;
  
  std::size_t prev_idx = 0;
  for (std::size_t i=1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
  {
    // check separation to the previous via-point inserted
    if (distance_points2d( transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position ) < min_separation)
      continue;
        
    // add via-point
    via_points_.push_back( Eigen::Vector2d( transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y ) );
    prev_idx = i;
  }
  
}
      
Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
  Eigen::Vector2d vel;
  vel.coeffRef(0) = std::sqrt( tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY() );
  vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
  return vel;
}
      
      
bool TebLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
  if (global_plan.empty())
    return true;
  
  try
  {
    // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
    geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
    geometry_msgs::PoseStamped robot;
    tf2::doTransform(global_pose, robot, global_to_plan_transform);
    
    double dist_thresh_sq = dist_behind_robot*dist_behind_robot;
    
    // iterate plan until a pose close the robot is found
    std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
    std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
    while (it != global_plan.end())
    {
      double dx = robot.pose.position.x - it->pose.position.x;
      double dy = robot.pose.position.y - it->pose.position.y;
      double dist_sq = dx * dx + dy * dy;
      if (dist_sq < dist_thresh_sq)
      {
         erase_end = it;
         break;
      }
      ++it;
    }
    if (erase_end == global_plan.end())
      return false;
    
    if (erase_end != global_plan.begin())
      global_plan.erase(global_plan.begin(), erase_end);
  }
  catch (const tf::TransformException& ex)
  {
    ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
    return false;
  }
  return true;
}
      

bool TebLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                  const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap, const std::string& global_frame, double max_plan_length,
                  std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx, geometry_msgs::TransformStamped* tf_plan_to_global) const
{
  // this method is a slightly modified version of base_local_planner/goal_functions.h

  const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

  transformed_plan.clear();

  try 
  {
    if (global_plan.empty())
    {
      ROS_ERROR("Received plan with zero length");
      *current_goal_idx = 0;
      return false;
    }

    // get plan_to_global_transform from plan frame to global_frame
    geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp,
                                                                                  plan_pose.header.frame_id, ros::Duration(cfg_.robot.transform_tolerance));

    //let's get the pose of the robot in the frame of the plan
    geometry_msgs::PoseStamped robot_pose;
    tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

    //we'll discard points on the plan that are outside the local costmap
    double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                     costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
    dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                           // located on the border of the local costmap
    

    int i = 0;
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist = 1e10;
    
    //we need to loop to a point on the plan that is within a certain distance of the robot
    bool robot_reached = false;
    for(int j=0; j < (int)global_plan.size(); ++j)
    {
      double x_diff = robot_pose.pose.position.x - global_plan[j].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[j].pose.position.y;
      double new_sq_dist = x_diff * x_diff + y_diff * y_diff;

      if (robot_reached && new_sq_dist > sq_dist)
        break;

      if (new_sq_dist < sq_dist) // find closest distance
      {
        sq_dist = new_sq_dist;
        i = j;
        if (sq_dist < 0.05)      // 2.5 cm to the robot; take the immediate local minima; if it's not the global
          robot_reached = true;  // minima, probably means that there's a loop in the path, and so we prefer this
      }
    }
    
    geometry_msgs::PoseStamped newer_pose;
    
    double plan_length = 0; // check cumulative Euclidean distance along the plan
    
    //now we'll transform until points are outside of our distance threshold
    while(i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length<=0 || plan_length <= max_plan_length))
    {
      const geometry_msgs::PoseStamped& pose = global_plan[i];
      tf2::doTransform(pose, newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);

      double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
      double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
      sq_dist = x_diff * x_diff + y_diff * y_diff;
      
      // caclulate distance to previous pose
      if (i>0 && max_plan_length>0)
        plan_length += distance_points2d(global_plan[i-1].pose.position, global_plan[i].pose.position);

      ++i;
    }
        
    // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
    // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
    if (transformed_plan.empty())
    {
      tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

      transformed_plan.push_back(newer_pose);
      
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = int(global_plan.size())-1;
    }
    else
    {
      // Return the index of the current goal point (inside the distance threshold)
      if (current_goal_idx) *current_goal_idx = i-1; // subtract 1, since i was increased once before leaving the loop
    }
    
    // Return the transformation from the global plan to the global planning frame if desired
    if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
  }
  catch(tf::LookupException& ex)
  {
    ROS_ERROR("No Transform available Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ConnectivityException& ex) 
  {
    ROS_ERROR("Connectivity Error: %s\n", ex.what());
    return false;
  }
  catch(tf::ExtrapolationException& ex) 
  {
    ROS_ERROR("Extrapolation Error: %s\n", ex.what());
    if (global_plan.size() > 0)
      ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

    return false;
  }

  return true;
}   
      
      
double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan, const geometry_msgs::PoseStamped& local_goal,
              int current_goal_idx, const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
  int n = (int)global_plan.size();
  
  // check if we are near the global goal already
  if (current_goal_idx > n-moving_average_length-2)
  {
    if (current_goal_idx >= n-1) // we've exactly reached the goal
    {
      return tf2::getYaw(local_goal.pose.orientation);
    }
    else
    {
      tf2::Quaternion global_orientation;
      tf2::convert(global_plan.back().pose.orientation, global_orientation);
      tf2::Quaternion rotation;
      tf2::convert(tf_plan_to_global.transform.rotation, rotation);
      // TODO(roesmann): avoid conversion to tf2::Quaternion
      return tf2::getYaw(rotation *  global_orientation);
    }     
  }
  
  // reduce number of poses taken into account if the desired number of poses is not available
  moving_average_length = std::min(moving_average_length, n-current_goal_idx-1 ); // maybe redundant, since we have checked the vicinity of the goal before
  
  std::vector<double> candidates;
  geometry_msgs::PoseStamped tf_pose_k = local_goal;
  geometry_msgs::PoseStamped tf_pose_kp1;
  
  int range_end = current_goal_idx + moving_average_length;
  for (int i = current_goal_idx; i < range_end; ++i)
  {
    // Transform pose of the global plan to the planning frame
    tf2::doTransform(global_plan.at(i+1), tf_pose_kp1, tf_plan_to_global);

    // calculate yaw angle  
    candidates.push_back( std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y,
        tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x ) );
    
    if (i<range_end-1) 
      tf_pose_k = tf_pose_kp1;
  }
  return average_angles(candidates);
}
      
      
void TebLocalPlannerROS::saturateVelocity(double& vx, double& vy, double& omega, double max_vel_x, double max_vel_y, double max_vel_trans, double max_vel_theta, 
              double max_vel_x_backwards) const
{
  double ratio_x = 1, ratio_omega = 1, ratio_y = 1;
  // Limit translational velocity for forward driving
  if (vx > max_vel_x)
    ratio_x = max_vel_x / vx;
  
  // limit strafing velocity
  if (vy > max_vel_y || vy < -max_vel_y)
    ratio_y = std::abs(max_vel_y / vy);
  
  // Limit angular velocity
  if (omega > max_vel_theta || omega < -max_vel_theta)
    ratio_omega = std::abs(max_vel_theta / omega);
  
  // Limit backwards velocity
  if (max_vel_x_backwards<=0)
  {
    ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
  }
  else if (vx < -max_vel_x_backwards)
    ratio_x = - max_vel_x_backwards / vx;

  if (cfg_.robot.use_proportional_saturation)
  {
    double ratio = std::min(std::min(ratio_x, ratio_y), ratio_omega);
    vx *= ratio;
    vy *= ratio;
    omega *= ratio;
  }
  else
  {
    vx *= ratio_x;
    vy *= ratio_y;
    omega *= ratio_omega;
  }

  double vel_linear = std::hypot(vx, vy);
  if (vel_linear > max_vel_trans)
  {
    double max_vel_trans_ratio = max_vel_trans / vel_linear;
    vx *= max_vel_trans_ratio;
    vy *= max_vel_trans_ratio;
  }
}
     
     
double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
{
  if (omega==0 || v==0)
    return 0;
    
  double radius = v/omega;
  
  if (fabs(radius) < min_turning_radius)
    radius = double(g2o::sign(radius)) * min_turning_radius; 

  return std::atan(wheelbase / radius);
}
     

void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!", opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}  
   
   
void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped>& transformed_plan,  int& goal_idx)
{
    ros::Time current_time = ros::Time::now();
    
    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup && 
        goal_idx < (int)transformed_plan.size()-1 && // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
       (no_infeasible_plans_>0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration )) // keep short horizon for at least a few seconds
    {
        ROS_INFO_COND(no_infeasible_plans_==1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);


        // Shorten horizon if requested
        // reduce to 50 percent:
        int horizon_reduction = goal_idx/2;
        
        if (no_infeasible_plans_ > 9)
        {
            ROS_INFO_COND(no_infeasible_plans_==10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
            horizon_reduction /= 2;
        }
        
        // we have a small overhead here, since we already transformed 50% more of the trajectory.
        // But that's ok for now, since we do not need to make transformGlobalPlan more complex 
        // and a reduced horizon should occur just rarely.
        int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
        goal_idx -= horizon_reduction;
        if (new_goal_idx_transformed_plan>0 && goal_idx >= 0)
            transformed_plan.erase(transformed_plan.begin()+new_goal_idx_transformed_plan, transformed_plan.end());
        else
            goal_idx += horizon_reduction; // this should not happen, but safety first ;-) 
    }
    
    
    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
        double max_vel_theta;
        double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
        if (cfg_.robot.min_turning_radius!=0 && max_vel_current>0)
            max_vel_theta = std::max( max_vel_current/std::abs(cfg_.robot.min_turning_radius),  cfg_.robot.max_vel_theta );
        else
            max_vel_theta = cfg_.robot.max_vel_theta;
        
        failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);
        
        bool oscillating = failure_detector_.isOscillating();
        bool recently_oscillated = (ros::Time::now()-time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently
        
        if (oscillating)
        {
            if (!recently_oscillated)
            {
                // save current turning direction
                if (robot_vel_.angular.z > 0)
                    last_preferred_rotdir_ = RotType::left;
                else
                    last_preferred_rotdir_ = RotType::right;
                //ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
            }
            time_last_oscillation_ = ros::Time::now();  
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
        }
        else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
        {
            last_preferred_rotdir_ = RotType::none;
            planner_->setPreferredTurningDir(last_preferred_rotdir_);
            //ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
        }
    }

}
     
void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
  boost::mutex::scoped_lock l(custom_obst_mutex_);
  custom_obstacle_msg_ = *obst_msg;  
}

void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
  ROS_INFO_ONCE("Via-points received. This message is printed once.");
  if (cfg_.trajectory.global_plan_viapoint_sep > 0)
  {
    //ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
     //        "Ignoring custom via-points.");
    custom_via_points_active_ = false;
    return;
  }

  boost::mutex::scoped_lock l(via_point_mutex_);
  via_points_.clear();
  for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
  {
    via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  custom_via_points_active_ = !via_points_.empty();
}
     
RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh, const TebConfig& config)
{
  std::string model_name; 
  if (!nh.getParam("footprint_model/type", model_name))
  {
    ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
    return boost::make_shared<PointRobotFootprint>();
  }
    
  // point  
  if (model_name.compare("point") == 0)
  {
    ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
    return boost::make_shared<PointRobotFootprint>(config.obstacles.min_obstacle_dist);
  }
  
  // circular
  if (model_name.compare("circular") == 0)
  {
    // get radius
    double radius;
    if (!nh.getParam("footprint_model/radius", radius))
    {
      ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/radius' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
    return boost::make_shared<CircularRobotFootprint>(radius);
  }
  
  // line
  if (model_name.compare("line") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get line coordinates
    std::vector<double> line_start, line_end;
    nh.getParam("footprint_model/line_start", line_start);
    nh.getParam("footprint_model/line_end", line_end);
    if (line_start.size() != 2 || line_end.size() != 2)
    {
      ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
    ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                     << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
    return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()), config.obstacles.min_obstacle_dist);
  }
  
  // two circles
  if (model_name.compare("two_circles") == 0)
  {
    // check parameters
    if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") 
        || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
    {
      ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                       << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    double front_offset, front_radius, rear_offset, rear_radius;
    nh.getParam("footprint_model/front_offset", front_offset);
    nh.getParam("footprint_model/front_radius", front_radius);
    nh.getParam("footprint_model/rear_offset", rear_offset);
    nh.getParam("footprint_model/rear_radius", rear_radius);
    ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius 
                    << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
    return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
  }

  // polygon
  if (model_name.compare("polygon") == 0)
  {

    // check parameters
    XmlRpc::XmlRpcValue footprint_xmlrpc;
    if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not exist. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    // get vertices
    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      try
      {
        Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
        ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
        return boost::make_shared<PolygonRobotFootprint>(polygon);
      } 
      catch(const std::exception& ex)
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }
    else
    {
      ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace() 
                       << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
      return boost::make_shared<PointRobotFootprint>();
    }
    
  }
  
  // otherwise
  ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
  return boost::make_shared<PointRobotFootprint>();
}       
       
       
Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
{
   // Make sure we have an array of at least 3 elements.
   if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
       footprint_xmlrpc.size() < 3)
   {
     ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
     throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                              "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
   }
 
   Point2dContainer footprint;
   Eigen::Vector2d pt;
 
   for (int i = 0; i < footprint_xmlrpc.size(); ++i)
   {
     // Make sure each element of the list is an array of size 2. (x and y coordinates)
     XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
     if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         point.size() != 2)
     {
       ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                               "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
    }

    pt.x() = getNumberFromXMLRPC(point[ 0 ], full_param_name);
    pt.y() = getNumberFromXMLRPC(point[ 1 ], full_param_name);

    footprint.push_back(pt);
  }
  return footprint;
}

double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  // Make sure that the value we're looking at is either a double or an int.
  if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
      value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
  {
    std::string& value_string = value;
    ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
               full_param_name.c_str(), value_string.c_str());
     throw std::runtime_error("Values in the footprint specification must be numbers");
   }
   return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

} // end namespace teb_local_planner

