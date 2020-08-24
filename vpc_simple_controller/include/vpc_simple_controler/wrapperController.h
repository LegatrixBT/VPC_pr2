/*
 * File: wrapperController.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Thursday, 4th June 2020 1:50:00 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Wednesday, 12th August 2020 1:04:49 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






/// ---------------------- HEADERS ------------------------- ///
#ifndef WRAPPERCONTROLLER_CLASS_H
#define WRAPPERCONTROLLER_CLASS_H

// #include <vector>
#include <iostream>

#include <ros/ros.h>

// include the usefull classes here VPC controller
#include "./VPCController.h"
#include <pr2_forearm_vison/visualFeatures.h>
// include the arms_traj pkg to perform movements
#include <pr2_arms_joint_trajectory/arms_joint_traj.h>


namespace Wrapper{

  /// ---------------------- DEFINES ------------------------- ///

  typedef VPCCtrl::VPCController Controller;

  /// ------------------------ CLASS -------------------------- ///

  class WrapperController{

    public:

     /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///

      WrapperController(ros::NodeHandle *nh);
      ~WrapperController();

      void callbackVF_l(const pr2_forearm_vison::visualFeatures::ConstPtr& msg);
      void callbackVF_r(const pr2_forearm_vison::visualFeatures::ConstPtr& msg);

      float getTs() const;

      // loop to manage the Visual Servoing
      void controlLoop();

    /// ---------------------------- Attributes --------------------------------------- ///

    ////////// TO PUT HERE AGAIN private:

    // ------------------------- ROS RELATED ----------------------- // 
      
      // To stay put with time
      ros::Timer timerControl;

      // --------- Publishers and subcribers and related ----------- //
      ros::AsyncSpinner* VF_spinner;
      // define user callback queue
      ros::CallbackQueue VF_queue;
      // Subscribers to the image streams  
      ros::Subscriber img_left_sub;
      ros::Subscriber img_right_sub;
      // Subscriber option object 
      ros::SubscribeOptions l_ops;
      ros::SubscribeOptions r_ops;

      // To convert the ros msgs to the class
      float* l_bufferVF;
      float* l_depthVF;

      float* r_bufferVF;
      float* r_depthVF;

      // Goal publishers for each arms 

      Arms_joint_traj::RobotArms* arms;


     // --------------------- CONTROLLER RELATED -------------------- //
      
      // The controller
      Controller vpc;

      // states and control

      float ts;

      bool isLAvailable, isRAvailable;
      bool isVS_ON;

      double* l_command;
      double* r_command;
    
  };

}
#endif // WRAPPERCONTROLLER_CLASS_H

