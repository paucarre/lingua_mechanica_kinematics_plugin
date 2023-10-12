
#include <ros/ros.h>
#include <urdf/model.h>
#include <tf_conversions/tf_kdl.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <lingua_mechanica_kinematics_plugin.hpp>
#include <limits>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <torch/script.h>

namespace lingua_mechanica_kinematics_plugin
{

bool LinguaMechanicaKinematicsPlugin::initialize(const std::string &robot_description,
    const std::string& group_name,
    const std::string& base_name,
    const std::string& tip_name,
    double search_discretization)
{

  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("lingua_mechanica");

  std::vector<std::string> tip_names = {tip_name};
  setValues(robot_description, group_name, base_name, tip_names, search_discretization);

  ros::NodeHandle node_handle("~");

  urdf::Model robot_model;
  std::string xml_string;

  std::string urdf_xml, full_urdf_xml;
  node_handle.param("urdf_xml", urdf_xml, robot_description);
  node_handle.searchParam(urdf_xml, full_urdf_xml);

  ROS_DEBUG_NAMED("lingua mechanica", "Reading xml file from parameter server");
  if (!node_handle.getParam(full_urdf_xml, xml_string))
  {
    ROS_FATAL_NAMED("lingua mechanica", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
    return false;
  }

  node_handle.param(full_urdf_xml, xml_string, std::string());
  robot_model.initString(xml_string);

  ROS_DEBUG_STREAM_NAMED("lingua mechanica", "Reading joints and links from URDF");

  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
  {
    ROS_FATAL("Failed to extract kdl tree from xml robot description");
    return false;
  }

  if (!tree.getChain(base_name, tip_name, chain))
  {
    ROS_FATAL("Couldn't find chain %s to %s", base_name.c_str(), tip_name.c_str());
    return false;
  }

  num_joints_ = chain.getNrOfJoints();

  std::vector<KDL::Segment> chain_segs = chain.segments;

  urdf::JointConstSharedPtr joint;

  std::vector<double> l_bounds, u_bounds;

  joint_min.resize(num_joints_);
  joint_max.resize(num_joints_);

  uint joint_num = 0;
  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {

    link_names_.push_back(chain_segs[i].getName());
    joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
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

  ROS_INFO_NAMED("lingua mechanica plugin", "Looking in common namespaces for param name: %s", (group_name + "/position_only_ik").c_str());
  lookupParam(group_name + "/position_only_ik", position_ik_, false);
  ROS_INFO_NAMED("lingua mechanica plugin", "Looking in common namespaces for param name: %s", (group_name + "/solve_type").c_str());
  lookupParam(group_name + "/solve_type", solve_type, std::string("Speed"));
  ROS_INFO_NAMED("lingua mechanica plugin", "Using solve type %s", solve_type.c_str());

  active_ = true;
  return true;
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
    ROS_ERROR_NAMED("lingua mechanica", "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != num_joints_)
  {
    ROS_ERROR_NAMED("lingua mechanica", "Joint angles vector must have size: %d", num_joints_);
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
    ROS_DEBUG_NAMED("lingua mechanica", "End effector index: %d", getKDLSegmentIndex(link_names[i]));
    if (fk_solver.JntToCart(jnt_pos_in, p_out, getKDLSegmentIndex(link_names[i])) >= 0)
    {
      tf::poseKDLToMsg(p_out, poses[i]);
    }
    else
    {
      ROS_ERROR_NAMED("lingua mechanica", "Could not compute FK for %s", link_names[i].c_str());
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
  ROS_DEBUG_STREAM_NAMED("lingua mechanica", "getPositionIK");

  if (!active_)
  {
    ROS_ERROR("kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  if (ik_seed_state.size() != num_joints_)
  {
    ROS_ERROR_STREAM_NAMED("lingua mechanica", "Seed state must have size " << num_joints_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }
  torch::Tensor tensor = torch::tensor({ik_pose.position.x, ik_pose.position.y, ik_pose.position.z, 
    ik_pose.orientation.x, ik_pose.orientation.y, ik_pose.orientation.z, ik_pose.orientation.w});

  //KDL::Frame frame;
  //tf::poseMsgToKDL(ik_pose, frame);
  //KDL::JntArray in(num_joints_), out(num_joints_);

  //for (uint z = 0; z < num_joints_; z++)
  //  in(z) = ik_seed_state[z];

  //KDL::Twist bounds = KDL::Twist::Zero();

  //if (position_ik_)
  //{
  //  bounds.rot.x(std::numeric_limits<float>::max());
  //  bounds.rot.y(std::numeric_limits<float>::max());
  //  bounds.rot.z(std::numeric_limits<float>::max());
  //}
  
  //double epsilon = 1e-5;  //Same as MoveIt's KDL plugin

  //lingua mechanica::SolveType solvetype;

  //if (solve_type == "Manipulation1")
  //  solvetype = lingua mechanica::Manip1;
  //else if (solve_type == "Manipulation2")
  //  solvetype = lingua mechanica::Manip2;
  //else if (solve_type == "Distance")
  //  solvetype = lingua mechanica::Distance;
  //else
  // {
  //  if (solve_type != "Speed")
  //  {
  //    ROS_WARN_STREAM_NAMED("lingua mechanica", solve_type << " is not a valid solve_type; setting to default: Speed");
  //  }
  //  solvetype = lingua mechanica::Speed;
  //}

  //lingua mechanica::lingua mechanica ik_solver(chain, joint_min, joint_max, timeout, epsilon, solvetype);
  //int rc = ik_solver.CartToJnt(in, frame, out, bounds);

  // TODO: placeholder for inverse kinematics

  //solution.resize(num_joints_);

  //if (rc >= 0)
  //{
  //  for (uint z = 0; z < num_joints_; z++)
  //    solution[z] = out(z);

    // check for collisions if a callback is provided
  //  if (!solution_callback.empty())
  //  {
  //    solution_callback(ik_pose, solution, error_code);
  //    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
  //    {
  //      ROS_DEBUG_STREAM_NAMED("lingua mechanica", "Solution passes callback");
  //      return true;
  //    }
  //    else
  //    {
  //      ROS_DEBUG_STREAM_NAMED("lingua mechanica", "Solution has error code " << error_code);
  //      return false;
  //    }
  //  }
  //  else
  //    return true; // no collision check callback provided
  //}

  //error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  solution.push_back(0.1);
  solution.push_back(0.2);
  solution.push_back(0.3);
  solution.push_back(0.4);
  solution.push_back(0.5);
  solution.push_back(0.6);
  error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  return true;
}



} // end namespace

// register LinguaMechanicaKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lingua_mechanica_kinematics_plugin::LinguaMechanicaKinematicsPlugin, kinematics::KinematicsBase);