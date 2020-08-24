/*
 * File: arms_geo_model.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Friday, 5th June 2020 12:00:15 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Sunday, 9th August 2020 3:45:45 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */







/// ---------------------- HEADERS ------------------------- ///
#ifndef ARMSGEOMODEL_CLASS_H
#define ARMSGEOMODEL_CLASS_H

//include of Eigen and mathlib
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <vector>


namespace Kinematic{

  /// ---------------------- DEFINES ------------------------- ///


  /// ------------------------ CLASS -------------------------- ///
  class GeometricModel{

    private:

      /// ---------------------------- Attributes --------------------------------------- ///

      // --------------------- ROBOT SPECIFICATIONS ----------------- //

      // From Willow Garage pr2 specification
      const double forearm_offset     = 0.321;
      const double shoulder_offset    = 0.1;
      const double upper_arm_offset   = 0.4;
  
      // Distances to be evaluated more carefully in the future (default values here) 
      // rosrun tf tf_echo /r_wrist_roll_link /r_gripper_tool_frame
      // - Translation: [0.180, 0.000, 0.000]
      const double link7_end_eff_offset  = 0.180;
  
      // Distance between torso_lift_link and first joint  can be mesure using
      // $ rosrun tf tf_echo /torso_lift_link /r_shoulder_pan_link 
      // - Translation: [0.000 , 0.188, 0.000]
      const double torso_link1_offset    = 0.188;
  
      // ------------------------- H MATRIX ----------------------- // 

      Eigen::Matrix4d T01_, T12_, T23_, T34_, T45_, T56_, T67_;
      // HTransforms between different joints of the robot that are not directly in the kinematic chain
      Eigen::Matrix4d T7EE_, T5C_, TB0_, TBC_, TC5_;
      // HTransforms between joint0 and jointX 
      Eigen::Matrix4d T02_, T03_, T04_, T05_, T06_, T07_, T0EE_; // to verify the model
      Eigen::Matrix4d TCB_, TBEE_, TCEE_, TEEB_, TEEC_;
      Eigen::Matrix4Xd TH_; 

    public:
    /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///
      GeometricModel();
  
      ~GeometricModel();
  
  
      // Define the homogeneous matrix of the pr2_robot according to the joint construction in the urdf file
      // and visualisable using Rviz by showing the TF of joints : 
      // {"r_shoulder_pan_link","r_shoulder_lift_link", "r_upper_arm_roll_link", "r_elbow_flex_link", ...
      //   ...  "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"}
      // *
      // with q the joint angle in radian of each joint and 
      // right_arm = true if workin on the right arm
      void evaluate(std::vector<double>& q, const bool right_arm);
      
      // Define the homogeneous matrix of the pr2_robot according to the joint construction in the urdf file
      // and visualisable using Rviz by showing the TF of joints : 
      // {"r_shoulder_pan_link","r_shoulder_lift_link", "r_upper_arm_roll_link", "r_elbow_flex_link", ...
      //   ...  "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"}
      // *
      // with q the joint angle in radian of each joint and 
      // right_arm = true if workin on the right arm
      void evaluate(Eigen::VectorXd& q, const bool right_arm);
      void evaluate5Joints(Eigen::VectorXd& q, const bool right_arm);
  
      // Give the values of the angles using the RPY formalism for homogeneous matrix
      void getPRY(Eigen::RowVector3d& RPY, const Eigen::Matrix4d TH ) const;
  
      // Get the H matrix H={H01 ,H02, ... H0EE}
      Eigen::Matrix4Xd getH() const;
  
      /// -------------------------- Attributes R-ONLY ------------------------------------ ///
      
      // This way does not make any copy, but returns a reference to const (time saver)
      // lecture only
      const Eigen::Matrix4d& T01()  const { return T01_; } 
      const Eigen::Matrix4d& T02()  const { return T02_; } 
      const Eigen::Matrix4d& T03()  const { return T03_; } 
      const Eigen::Matrix4d& T04()  const { return T04_; } 
      const Eigen::Matrix4d& T05()  const { return T05_; } 
      const Eigen::Matrix4d& T06()  const { return T06_; } 
      const Eigen::Matrix4d& T07()  const { return T07_; } 
      const Eigen::Matrix4d& T0EE() const { return T0EE_; } 
      const Eigen::Matrix4d& T12()  const { return T12_; } 
      const Eigen::Matrix4d& T23()  const { return T23_; } 
      const Eigen::Matrix4d& T34()  const { return T34_; } 
      const Eigen::Matrix4d& T45()  const { return T45_; } 
      const Eigen::Matrix4d& T56()  const { return T56_; } 
      const Eigen::Matrix4d& T67()  const { return T67_; } 
      const Eigen::Matrix4d& T7EE() const { return T7EE_; }
      const Eigen::Matrix4d& T5C()  const { return T5C_; }
      const Eigen::Matrix4d& TC5()  const { return TC5_; }
      const Eigen::Matrix4d& TBC()  const { return TBC_; }
      const Eigen::Matrix4d& TCB()  const { return TCB_; }
      const Eigen::Matrix4d& TB0()  const { return TB0_; }
      const Eigen::Matrix4d& TBEE() const { return TBEE_; }
      const Eigen::Matrix4d& TEEB() const { return TEEB_; }
      const Eigen::Matrix4d& TCEE() const { return TCEE_; }
      const Eigen::Matrix4d& TEEC() const { return TEEC_; }
      // Not possible to turn as a const due to the dynamic nature --> getter then as a const
      //const Eigen::Matrix4d& TH()   const { return TH_; } 
  };


}

#endif // ARMSGEOMODEL_CLASS_H