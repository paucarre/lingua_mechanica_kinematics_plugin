#ifndef LINGUA_MECHANICA_KINEMATICS_PLUGIN_
#define LINGUA_MECHANICA_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// System
#include <memory>

// ROS msgs
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/robot_state.h>


#include <moveit/kinematics_base/kinematics_base.h>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit_msgs/KinematicSolverInfo.h>

namespace lingua_mechanica_kinematics_plugin
{

class LinguaMechanicaKinematicsPlugin : public kinematics::KinematicsBase
{
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  uint num_joints_;
  bool active_; // Internal variable that indicates whether solvers are configured and ready

  KDL::Chain chain;
  bool position_ik_;

  KDL::JntArray joint_min, joint_max;

  std::string solve_type;

public:
  const std::vector<std::string>& getJointNames() const
  {
    return joint_names_;
  }
  const std::vector<std::string>& getLinkNames() const
  {
    return link_names_;
  }


  /** @class
   *  @brief Interface for a kinematics plugin
   */
  LinguaMechanicaKinematicsPlugin(): active_(false), position_ik_(false) {}

  ~LinguaMechanicaKinematicsPlugin()
  {
  }

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @return True if a valid solution was found, false otherwise
   */

  // Returns the first IK solution that is within joint limits, this is called by get_ik() service
  bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                     const std::vector<double> &ik_seed_state,
                     std::vector<double> &solution,
                     moveit_msgs::MoveItErrorCodes &error_code,
                     const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).  The consistency_limit specifies that only certain redundancy positions
   * around those specified in the seed state are admissible and need to be searched.
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param consistency_limit the distance that the redundancy can be from the current position
   * @return True if a valid solution was found, false otherwise
   */
  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        const std::vector<double> &consistency_limits,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  bool searchPositionIK(const geometry_msgs::Pose &ik_pose,
                        const std::vector<double> &ik_seed_state,
                        double timeout,
                        std::vector<double> &solution,
                        const IKCallbackFn &solution_callback,
                        moveit_msgs::MoveItErrorCodes &error_code,
                        const std::vector<double> &consistency_limits,
                        const kinematics::KinematicsQueryOptions &options) const;



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
  bool getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                             std::vector<std::vector<double> >& solutions, kinematics::KinematicsResult& result,
                             const kinematics::KinematicsQueryOptions& options) const;

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   *
   * This FK routine is only used if 'use_plugin_fk' is set in the 'arm_kinematics_constraint_aware' node,
   * otherwise ROS TF is used to calculate the forward kinematics
   *
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  bool getPositionFK(const std::vector<std::string> &link_names,
                     const std::vector<double> &joint_angles,
                     std::vector<geometry_msgs::Pose> &poses) const;


  bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                          const std::string& base_frame, const std::vector<std::string>& tip_frames,                          
                          double search_discretization);
                          
private:
  moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

  int getKDLSegmentIndex(const std::string &name) const;
  
  std::shared_ptr<ros::ServiceClient> ik_service_client_;

  unsigned int dimension_; /** Dimension of the group */

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr robot_state_;

  int num_possible_redundant_joints_;

}; // end class
}

#endif
