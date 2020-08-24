/*
 * File: VPCController
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Wednesday, 3rd June 2020 11:15:11 am
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Thursday, 13th August 2020 9:04:42 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */







#ifndef VPCCONTROLLER_CLASS_H
#define VPCCONTROLLER_CLASS_H

/// ---------------------- HEADERS ------------------------- ///

#include <vector>
#include <iostream>

// #include <Eigen/Core>
// #include <Eigen/Dense>

// #include <Nlopt/nlopt.h>
#include <Nlopt/nlopt.hpp>
#include <pr2_prediction_model/globalModel.h>

namespace VPCCtrl{

  /// ---------------------- DEFINES ------------------------- ///

  #define NB_JOINTS 5
  #define ERROR_VAL_STOP_VPC 0.035


  /// ------------------------ CLASS -------------------------- ///

  
  class VPCController{

      public:

      /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///

        VPCController();
        ~VPCController();

        // Setup the optimization solver properly
        const void setupSolver();

        // Set the reference visual features
        const void setVF_ref(const bool r_arm, 
                       double x0, double y0, double x1, double y1,
                       double x2, double y2, double x3, double y3);

        // Set the current VF as ref_VF
        const void setVF_as_VF_ref(const bool r_arm);

        // Set the state for an arm
        const void set_state(const bool r_arm, std::vector<double>& state);

        // Get the sampling time
        float getTs() const;

        // Get the number of features
        int getNbFeatures() const;

        // Return true if the task is considered achieved, 
        // if the sum of the squared error is under the ERROR_VAL_STOP_VPC value
        
        // Update the depth visual features values
        const void updateDepthVF(const bool r_arm, float* z);

        // Update the visuals features values
        const void updateVF(const bool r_arm, float* vf);

        bool taskAchieved(const bool r_arm) const;

        void vpc(const bool r_arm, Eigen::VectorXd* q);

        // static double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
  

     /// ---------------------------- Attributes --------------------------------------- ///

      //private:

        // Sampling time
        double ts;

        int nbPredictions;
        int nbFeatures;
        int nbJoints;

        // ----------- Robot attributes ----------- //
        // Focal length of the forearm cameras 
        float focal;


            // --- variables of the robot --- //

        // Positions : q0-->q4
        std::vector<double> l_state; 
        Eigen::VectorXd l_q_state; 
        // Commands : vx, vy, vz, wx, wy, wz
        Eigen::VectorXd l_command;

        // Positions : x, y, z, roll, pitch, yaw
        std::vector<double> r_state; 
        // Commands : vx, vy, vz, wx, wy, wz
        Eigen::VectorXd r_command;

        // ------------ Visual Servoing attributes ---------- //

        Eigen::VectorXd l_VF;     // Visual features vector
        Eigen::VectorXd l_VF_ref; // Reference visual features vector
        Eigen::VectorXd l_VF_depth;    // Visual depth features vector

        Eigen::VectorXd r_VF;     // Visual features vector
        Eigen::VectorXd r_VF_ref; // Reference visual features vector
        Eigen::VectorXd r_VF_depth;    // Visual depth features vector
        
        // True if the depth is acquire by the forearm cameras 
        // false is depth is a constant and zConst value is feeded
        bool l_zReal;
        float l_zConst;

        bool r_zReal;
        float r_zConst;  

        // -------------- Solver  attributes -------------- //

        GlobalModel *gModel;
        nlopt::opt *opt;
        double minf;
        double stop_val;
        double VPC_stop;

        std::vector<double> v_lb;  // lower bound
        std::vector<double> v_ub;  // upper bound

        std::vector<double>* std_l_x; // states to optimize
        std::vector<double>* std_l_x_prev; // previous states optimized for wrm stating

        std::vector<double>* std_r_x; // states to optimize
        std::vector<double>* std_r_x_prev; // previous states optimized for wrm stating

  };
}



#endif // VPCCONTROLLER_CLASS_H