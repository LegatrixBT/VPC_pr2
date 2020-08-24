/*
 * File: pr2_joint_states_listener_client.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Saturday, 16th May 2020 11:46:28 am
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Thursday, 11th June 2020 7:18:21 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */







#ifndef PR2_JOINT_STATES_LISTENER_CLIENT_CLASS_H
#define PR2_JOINT_STATES_LISTENER_CLIENT_CLASS_H

/// ---------------------- HEADERS ------------------------- ///

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Include of the custom server for listen the joint_state topic 
#include <pr2_joint_states_listener/pr2_joint_states_listener.h>


namespace LastestJointStates{
  /// ----------------------- DEFINES ------------------------- ///
  
  #define JOINTS_NAME_PR2_ARMS {"r_shoulder_pan_joint","r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint", "l_shoulder_pan_joint","l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"}
  #define JOINTS_NAME_PR2_R_ARMS {"r_shoulder_pan_joint","r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"}
  #define JOINTS_NAME_PR2_L_ARMS {"l_shoulder_pan_joint","l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"}
  #define JOINTS_NAME_PR2_FOREARMS {"r_shoulder_pan_joint","r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "l_shoulder_pan_joint","l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint"}
  #define JOINTS_NAME_PR2_R_FOREARMS {"r_shoulder_pan_joint","r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint"}
  #define JOINTS_NAME_PR2_L_FOREARMS {"l_shoulder_pan_joint","l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint"}

  // According to the size of the joints_name_pr2_*
  #define NB_JOINTS_PR2_ARMS 14
  #define NB_JOINTS_PR2_UNIQUE_ARM 7
  #define NB_JOINTS_PR2_FOREARMS 10
  #define NB_JOINTS_PR2_UNIQUE_FOREARM 5
  
  #define JOINT_POSITION 0
  #define JOINT_VELOCITY 1
  #define JOINT_EFFORT 2
  
  // According to the number of joint_(parameter)
  #define NB_JOINT_STATES 3
  
  
  /// ------------------------ CLASS -------------------------- ///
  
      
  class LastestJointStatesClient{
  
  
      private:
      
      // Service object that will be used to contact the server        
      pr2_joint_states_listener::pr2_joint_states_listener service;
      // Client object that will ask a service to the server
      ros::ServiceClient client;
  
      public: 
  
      /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///
  
      // Constructor of the class LastestJointStatesClient
      LastestJointStatesClient(ros::NodeHandle *nh);
  
      // Destructor of the class LastestJointStatesClient
      ~LastestJointStatesClient();
  
      // (Faster) Request (of both the arms joints) to the service server the states of each articulation and store the result in a static array 
      void call_serivce_last_arms_joint_states(std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES>& joint_states);
  
      /// ------------------------------------------------------------------------------------------------ ///
  
      // (Faster) Request (of both the forearms joints) to the service server the states of each articulation and store the result in a static array 
      void call_serivce_last_forearms_joint_states(std::array<std::array<double,NB_JOINTS_PR2_FOREARMS>,NB_JOINT_STATES>& joint_states);
  
      /// ------------------------------------------------------------------------------------------------ ///
  
      // (Faster) Request (of both the forearms joints) to the service server the position of each articulation and store the result in a static array 
      void call_serivce_last_forearms_joint_position(std::array<double,NB_JOINTS_PR2_FOREARMS>& joint_positions);
  
      /// ------------------------------------------------------------------------------------------------ ///
  
      // (Faster) Request (of both the forearms joints) to the service server the velocity of each articulation and store the result in a static array 
      void call_serivce_last_forearms_joint_velocity(std::array<double,NB_JOINTS_PR2_FOREARMS>& joint_velocities);
  
      /// ------------------------------------------------------------------------------------------------ ///
  
      // (Longer) Generic (joint) request to the service server the states of the joint_names passed as parameter and store the result in a vector
      std::vector<std::vector<double>> call_serivce_last_joint_states(std::vector<std::string> joint_names);
  
  };
}

#endif // ifndef PR2_JOINT_STATES_LISTENER_CLIENT_CLASS_H