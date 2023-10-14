
#include <ros/ros.h>
#include <urdf/model.h>
#include <tf_conversions/tf_kdl.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <lingua_mechanica_kinematics_plugin.hpp>
#include <lingua_mechanica_kinematics_msgs/GetPositionIK.h>
#include <limits>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <moveit_msgs/GetPositionIK.h>
#include <class_loader/class_loader.hpp>
#include <moveit/robot_state/conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace lingua_mechanica_kinematics_plugin
{

bool LinguaMechanicaKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                          const std::string& base_frame, const std::vector<std::string>& tip_frames,                          
                          double search_discretization)
{
  
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  ROS_DEBUG_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Reading joints and links from URDF");


  KDL::Tree tree;
  auto urdf = robot_model_->getURDF();
  if (!kdl_parser::treeFromUrdfModel(*urdf, tree))
  {
    ROS_FATAL("Failed to extract kdl tree from xml robot description");
    return false;
  }

  if (!tree.getChain(base_frame, tip_frames[0], chain))
  {
    ROS_FATAL("Couldn't find chain %s to %s", base_frame.c_str(), tip_frames[0].c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);

  const urdf::ModelInterfaceSharedPtr& urdf_robot_model = robot_model_->getURDF();
  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {

    link_names_.push_back(chain_segs[i].getName());
    joint = urdf_robot_model->getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      assert(joint_num <= num_joints_);
      float lower, upper;
      int hasLimits;
      joint_names_.push_back(joint->name);
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        joint_min(joint_num - 1) = lower;
        joint_max(joint_num - 1) = upper;
      }
      else
      {
        joint_min(joint_num - 1) = std::numeric_limits<float>::lowest();
        joint_max(joint_num - 1) = std::numeric_limits<float>::max();
      }
      ROS_INFO_STREAM("IK Using joint " << chain_segs[i].getName() << " " << joint_min(joint_num - 1) << " " << joint_max(joint_num - 1));
    }
  }




  ROS_INFO_NAMED("LinguaMechanicaKinematicsPlugin", "Looking in common namespaces for param name: %s", (group_name + "/position_only_ik").c_str());
  lookupParam(group_name + "/position_only_ik", position_ik_, false);
  ROS_INFO_NAMED("LinguaMechanicaKinematicsPlugin", "Looking in common namespaces for param name: %s", (group_name + "/solve_type").c_str());
  lookupParam(group_name + "/solve_type", solve_type, std::string("Speed"));
  ROS_INFO_NAMED("LinguaMechanicaKinematicsPlugin", "Using solve type %s", solve_type.c_str());

  ROS_INFO_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "LinguaMechanicaKinematicsPlugin initializing");
  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  // Get the dimension of the planning group
  dimension_ = joint_model_group_->getVariableCount();
  ROS_INFO_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Dimension planning group '"
                                   << group_name << "': " << dimension_
                                   << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
                                   << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
  }

  // Make sure all the tip links are in the link_names vector
  for (const std::string& tip_frame : tip_frames_)
  {
    if (!joint_model_group_->hasLinkModel(tip_frame))
    {
      ROS_ERROR_NAMED("LinguaMechanicaKinematicsPlugin", "Could not find tip name '%s' in joint group '%s'", tip_frame.c_str(), group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frame);
  }

  std::string ik_service_name;
  lookupParam("kinematics_solver_service_name", ik_service_name, std::string("solve_ik"));

  // Setup the joint state groups that we need
  robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
  robot_state_->setToDefaultValues();

  // Create the ROS service client
  ros::NodeHandle nonprivate_handle("");
  ik_service_client_ = std::make_shared<ros::ServiceClient>(
      nonprivate_handle.serviceClient<lingua_mechanica_kinematics_msgs::GetPositionIK>(ik_service_name));
  if (!ik_service_client_->waitForExistence(ros::Duration(0.1)))  // wait 0.1 seconds, blocking
    ROS_WARN_STREAM_NAMED("LinguaMechanicaKinematicsPlugin",
                          "Unable to connect to ROS service client with name: " << ik_service_client_->getService());
  else
    ROS_INFO_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Service client started with ROS service name: " << ik_service_client_->getService());

  active_ = true;
  ROS_DEBUG_NAMED("LinguaMechanicaKinematicsPlugin", "ROS service-based kinematics solver initialized");
  
  return active_;
}


int LinguaMechanicaKinematicsPlugin::getKDLSegmentIndex(const std::string &name) const
{
  int i = 0;
  while (i < (int)chain.getNrOfSegments())
  {
    if (chain.getSegment(i).getName() == name)
    {
      return i + 1;
    }
    i++;
  }
  return -1;
}


bool LinguaMechanicaKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
    const std::vector<double> &joint_angles,
    std::vector<geometry_msgs::Pose> &poses) const
{
  if (!active_)
  {
    ROS_ERROR_NAMED("LinguaMechanicaKinematicsPlugin", "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED("LinguaMechanicaKinematicsPlugin", "Joint angles vector must have size: %d", num_joints_);
    return false;
  }

  KDL::Frame p_out;
  geometry_msgs::PoseStamped pose;
  tf::Stamped<tf::Pose> tf_pose;

  KDL::JntArray jnt_pos_in(num_joints_);
  for (unsigned int i = 0; i < num_joints_; i++)
  {
    jnt_pos_in(i) = joint_angles[i];
  }

  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  bool valid = true;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    ROS_DEBUG_NAMED("LinguaMechanicaKinematicsPlugin", "End effector index: %d", getKDLSegmentIndex(link_names[i]));
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      tf::poseKDLToMsg(p_out, poses[i]);
    }
    else
    {
      ROS_ERROR_NAMED("LinguaMechanicaKinematicsPlugin", "Could not compute FK for %s", link_names[i].c_str());
      valid = false;
    }
  }

  return valid;
}


bool LinguaMechanicaKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          default_timeout_,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool LinguaMechanicaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool LinguaMechanicaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool LinguaMechanicaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool LinguaMechanicaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    const std::vector<double> &consistency_limits,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  return searchPositionIK(ik_pose,
                          ik_seed_state,
                          timeout,
                          solution,
                          solution_callback,
                          error_code,
                          consistency_limits,
                          options);
}

bool LinguaMechanicaKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose,
    const std::vector<double> &ik_seed_state,
    double timeout,
    std::vector<double> &solution,
    const IKCallbackFn &solution_callback,
    moveit_msgs::MoveItErrorCodes &error_code,
    const std::vector<double> &consistency_limits,
    const kinematics::KinematicsQueryOptions &options) const
{
  //TODO: this is a hack, make it properly generic
  const std::vector<geometry_msgs::Pose> ik_poses =  {ik_pose};

  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Seed state must have size " << num_joints_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  //torch::Tensor tensor = torch::tensor({ik_pose.position.x, ik_pose.position.y, ik_pose.position.z, 
  // ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w});

  lingua_mechanica_kinematics_msgs::GetPositionIK ik_srv;
  ik_srv.request.ik_request.avoid_collisions = true;
  ik_srv.request.ik_request.group_name = getGroupName();

  // Copy seed state into virtual robot state and convert into moveit_msg
  robot_state_->setJointGroupPositions(joint_model_group_, ik_seed_state);
  moveit::core::robotStateToRobotStateMsg(*robot_state_, ik_srv.request.ik_request.robot_state);

  // Load the poses into the request in difference places depending if there is more than one or not
  geometry_msgs::PoseStamped ik_pose_st;
  ik_pose_st.header.frame_id = base_frame_;
  if (tip_frames_.size() > 1)
  {
    // Load into vector of poses
    for (std::size_t i = 0; i < tip_frames_.size(); ++i)
    {
      ik_pose_st.pose = ik_poses[i];
      ik_srv.request.ik_request.pose_stamped_vector.push_back(ik_pose_st);
      ik_srv.request.ik_request.ik_link_names.push_back(tip_frames_[i]);
    }
  }
  else
  {
    ik_pose_st.pose = ik_poses[0];

    // Load into single pose value
    ik_srv.request.ik_request.pose_stamped = ik_pose_st;
    ik_srv.request.ik_request.ik_link_name = getTipFrames()[0];
  }


  ROS_DEBUG_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Calling service: " << ik_service_client_->getService());
  if (ik_service_client_->call(ik_srv))
  {
    // Check error code
    error_code.val = ik_srv.response.error_code.val;
    if (error_code.val != error_code.SUCCESS)
    {
      // TODO: check all solutions
      ROS_DEBUG_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "An IK that satisifes the constraints and is collision free could not be found."
                                        << "\nRequest was: \n"
                                        << ik_srv.request.ik_request << "\nResponse was: \n"
                                        << ik_srv.response.solutions.at(0));
      switch (error_code.val)
      {
        case moveit_msgs::MoveItErrorCodes::FAILURE:
          ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Service failed with with error code: FAILURE");
          break;
        case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
          ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Service failed with with error code: NO IK SOLUTION");
          break;
        default:
          ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Service failed with with error code: " << error_code.val);
      }
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM("Service call failed to connect to service: " << ik_service_client_->getService());
    error_code.val = error_code.FAILURE;
    return false;
  }
  // TODO: use arrays
  // Convert the robot state message to our robot_state representation
  // TODO: check all solutions
  if (!moveit::core::robotStateMsgToRobotState(ik_srv.response.solutions[0], *robot_state_))
  {
    ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin",
                           "An error occured converting received robot state message into internal robot state.");
    error_code.val = error_code.FAILURE;
    return false;
  }

  // Get just the joints we are concerned about in our planning group
  robot_state_->copyJointGroupPositions(joint_model_group_, solution);

  //TODO: callback for all the poses as a filter
  // Run the solution callback (i.e. collision checker) if available
  if (!solution_callback.empty())
  {
    ROS_DEBUG_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "Calling solution callback on IK solution");

    // hack: should use all poses, not just the 0th
    solution_callback(ik_poses[0], solution, error_code);

    if (error_code.val != error_code.SUCCESS)
    {
      switch (error_code.val)
      {
        case moveit_msgs::MoveItErrorCodes::FAILURE:
          ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "IK solution callback failed with with error code: FAILURE");
          break;
        case moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION:
          ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "IK solution callback failed with with error code: "
                                        "NO IK SOLUTION");
          break;
        default:
          ROS_ERROR_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "IK solution callback failed with with error code: " << error_code.val);
      }
      return false;
    }
  }

  ROS_INFO_STREAM_NAMED("LinguaMechanicaKinematicsPlugin", "IK Solver Succeeded!");
  return true;
}



/**
 * @brief Given the desired poses of all end-effectors, compute joint angles that are able to reach it.
 *
 * The default implementation returns only one solution and so its result is equivalent to calling
 * 'getPositionIK(...)' with a zero initialized seed.
 *
 * Some planners (e.g. IKFast) support getting multiple joint solutions for a single pose.
 * This can be enabled using the |DiscretizationMethods| enum and choosing an option that is not |NO_DISCRETIZATION|.
 *
 * @param ik_poses  The desired pose of each tip link
 * @param ik_seed_state an initial guess solution for the inverse kinematics
 * @param solutions A vector of valid joint vectors. This return has two variant behaviors:
 *                  1) Return a joint solution for every input |ik_poses|, e.g. multi-arm support
 *                  2) Return multiple joint solutions for a single |ik_poses| input, e.g. underconstrained IK
 *                  TODO(dave): This dual behavior is confusing and should be changed in a future refactor of this API
 * @param result A struct that reports the results of the query
 * @param options An option struct which contains the type of redundancy discretization used. This default
 *                implementation only supports the KinematicSearches::NO_DISCRETIZATION method; requesting any
 *                other will result in failure.
 * @return True if a valid set of solutions was found, false otherwise.
 */
bool LinguaMechanicaKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                            std::vector<std::vector<double> >& solutions, kinematics::KinematicsResult& result,
                            const kinematics::KinematicsQueryOptions& options) const {

  //TODO: implement
  return false;
}


} // end namespace

// register LinguaMechanicaKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lingua_mechanica_kinematics_plugin::LinguaMechanicaKinematicsPlugin, kinematics::KinematicsBase);