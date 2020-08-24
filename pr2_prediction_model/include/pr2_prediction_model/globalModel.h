/*
 * File: global.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Tuesday, 23rd June 2020 2:32:08 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Sunday, 23rd August 2020 7:21:43 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






/// ---------------------- HEADERS ------------------------- ///
#ifndef GLOBAL_MODEL_CLASS_H
#define GLOBAL_MODEL_CLASS_H

//include of Eigen and mathlib
// #include <Eigen/Core>
// #include <Eigen/Dense>
#include <vector>
#include <pr2_prediction_model/model.h>
// #include <pr2_geometric_model/arms_geo_model.h>
// #include<Nlopt/nlopt.hpp>
#include <iostream>
#include <iomanip> 
#include <chrono>

#define DEBUG(x) std::cout<< "debug num " << x << "\n";


struct Timer{
    std::chrono::time_point<std::chrono::steady_clock> start, end;
    std::chrono::duration<float> duration;

    Timer(){
        start = std::chrono::steady_clock::now();

    }
    ~Timer(){
        end = std::chrono::steady_clock::now();
        duration = end - start;

        float ms = duration.count() * 1000.0f;
        std::cout << "Timer took " <<  ms << "ms \n";
    }
};

  class GlobalModel : public Prediction::Model{
      
    /// ---------------------------- Attributes --------------------------------------- ///

    private:

    /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///

    public:
    // Use the constructor of Model
    using Model::Model;

    ~GlobalModel();

    
    struct my_func_data { //data structure to give to the solveur : contains only refs to existing objects 
    
      bool& r_arm;
      int& nbPredictions;
      int& nbFeatures;
      int& nbJoints;
      double& ts;
      float& focal;

      float& l_zConst;
      bool& z_real;
      Eigen::VectorXd& l_VF;
      Eigen::VectorXd& l_VF_ref;
      Eigen::VectorXd& l_VF_depth; 
      Eigen::VectorXd& l_q_state; 
      
  
      //! Constructor by ref to don't make copy of the existing objects and manipulate them
      my_func_data(bool* r_arm, int* nbPredictions, int* nbFeatures, int* nbJoints, double* ts, float* focal, float* l_zConst, bool* z_real,
                   Eigen::VectorXd* l_VF, Eigen::VectorXd* l_VF_ref, Eigen::VectorXd* l_VF_depth, Eigen::VectorXd* l_q_state) :
          r_arm(*r_arm), 
          nbPredictions(*nbPredictions),
          nbFeatures(*nbFeatures),
          nbJoints(*nbJoints),
          ts(*ts),
          focal(*focal),
          l_zConst(*l_zConst),
          z_real(*z_real),
          l_VF(*l_VF),
          l_VF_ref(*l_VF_ref),
          l_VF_depth(*l_VF_depth),
          l_q_state(*l_q_state)
          {}
  
      //! Destructor not really necessary because only refs, but better keep it
      ~my_func_data(){}
    };

    // hold the function data
    my_func_data* my_f_data;
    
    static double myfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data);

  };


#endif // GLOBAL_MODEL_CLASS_H