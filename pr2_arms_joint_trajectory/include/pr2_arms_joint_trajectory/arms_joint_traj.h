/*
 * File: arms_joint_traj.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Friday, 5th June 2020 6:39:05 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Friday, 14th August 2020 12:02:18 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






/// ---------------------- HEADERS ------------------------- ///
#ifndef ARMSJOINTTRAJ_CLASS_H
#define ARMSJOINTTRAJ_CLASS_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <pr2_controllers_msgs/JointControllerState.h>

// include of the joint_states_listener (as a dyn lib)
#include <pr2_joint_states_listener/pr2_joint_states_listener_client.h>
// include the kinematics
#include <pr2_geometric_model/arms_geo_model.h>


namespace Arms_joint_traj{

  /// ---------------------- DEFINES ------------------------- ///

  typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
  typedef LastestJointStates::LastestJointStatesClient JointStatesClient;


  // /// ------------------------ CLASS -------------------------- ///
  // Class that allow to move the pr2_arms using actionlib to feed the controllers
  // pr2_controllers_msgs::JointTrajectoryGoal is a lib that wrap the low level command to feed /r_arm_controller/command
  // with the type [trajectory_msgs/JointTrajectory]
  class RobotArms{

    public:
    /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///
      
      //! Initialize the action client and wait for action server to come up
      //! Initialize then the joint states client
      RobotArms(ros::NodeHandle* nh);
      
      //! Clean up the action clients and joint states client
      ~RobotArms();
  
      //! Sends the command to start a given trajectory for a given arm
      void startArmTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool r_arm, float delay);
      
      //! Sends the command to start a given trajectories for both arms
      void startBothTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goalR, pr2_controllers_msgs::JointTrajectoryGoal goalL, float delay);

      void startBothTrajectory(const float delay);

      //! Get the states of the arms and feed the both arrays with the current arms positions
      void get_arms_joint_position(std::array<double,NB_JOINTS_PR2_UNIQUE_ARM>& r_arm_pos, std::array<double,NB_JOINTS_PR2_UNIQUE_ARM>& l_arm_pos);
      
     
      //! Retrunr the goal that apply the command q choose by the user in the console for the right if r_arm(true) or left arm
      pr2_controllers_msgs::JointTrajectoryGoal arm_joint_movement_user_verif(bool r_arm);

      void verif_torse_ee(Kinematic::GeometricModel& model, const bool r_arm);

      //! Init the arm goal
      void init_arm_goal();

      //! Get the states of the arms and feed the both goals with the current arms positions
      void get_arms_joint_position(pr2_controllers_msgs::JointTrajectoryGoal& r_goal, pr2_controllers_msgs::JointTrajectoryGoal& l_goal);
 
      //! Get the states of the arms and feed the both arrays with the current arms velocities
      void get_arms_joint_velocities(pr2_controllers_msgs::JointTrajectoryGoal& r_goal, pr2_controllers_msgs::JointTrajectoryGoal& l_goal);

      //! Get the states of the arms and feed the both arrays with the current arms infos (position and velocities)
      void get_arms_joint_infos(pr2_controllers_msgs::JointTrajectoryGoal& r_goal, pr2_controllers_msgs::JointTrajectoryGoal& l_goal);

      //! Update the arms goal with the current arms infos (position and velocities)
      void update_arms_joint_infos();

      //! Get the state of unique arm (after update) and feed the array with the current arm info (position or velocities)
      std::vector<double> get_arm_joint_info(const bool r_arm, const bool position);

      //! Get the states of the arms (after update) and feed the both arrays with the current arms infos (position and velocities)
      const void get_arms_joints_info(std::vector<std::vector<double>>& l_values, std::vector<std::vector<double>>& r_values);
     
      //! Get the states of the arms (after update) and feed the both matrix with the current arms infos (position or velocities)
      const void get_arms_joints_info(const bool position, Eigen::VectorXd& l_values, Eigen::VectorXd& r_values);

      //! Set a goal a new goal for the arm 
      void set_arm_goal(std::vector<double>& q, const bool& r_arm);
      
      //! Set a goal a new goal for the arm 
      void set_arm_goal(const bool velocity, std::vector<double>& command, const bool& r_arm);

      // Overloading the << operator to print the current arms joints position
      friend std::ostream& operator<<(std::ostream& stream, const RobotArms& arms);

      //! Returns the current state of the right arm actionlib client
      actionlib::SimpleClientGoalState getRState();
      
      //! Returns the current state of the left arm actionlib client
      actionlib::SimpleClientGoalState getLState();

    private:
      // Action client for the joint trajectory action 
      // used to trigger the arm movement action
      TrajClient* right_arm_traj_client_;
      TrajClient* left_arm_traj_client_;
  
      // Joint_states_client for read position 
      // and velocities of joints
      JointStatesClient* joint_states_client_;

      // Keeping a goal object as memory of the previous move and to save time when moving to a new point
      pr2_controllers_msgs::JointTrajectoryGoal l_arm_goal;
      pr2_controllers_msgs::JointTrajectoryGoal r_arm_goal;

  };

}

#endif // ARMSJOINTTRAJ_CLASS_H