/*
 * File: model.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Tuesday, 23rd June 2020 1:41:57 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Friday, 14th August 2020 10:33:50 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






/// ---------------------- HEADERS ------------------------- ///
#ifndef MODEL_CLASS_H
#define MODEL_CLASS_H

//include of Eigen and mathlib
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pr2_geometric_model/arms_geo_model.h>


namespace Prediction{

  class Model{
      
    /// ---------------------------- Attributes --------------------------------------- ///

    protected:

    static Eigen::VectorXd C;
    static Eigen::VectorXd S_l;
    static Eigen::VectorXd S_ref_l;
    static Eigen::VectorXd Sm_l; 
    static Eigen::Matrix4d S_temp_l;
    static Eigen::MatrixXd temp_l_VF;
    static Eigen::VectorXd S_eval;
    static Eigen::MatrixXd Hi2cam;

    static Eigen::Matrix4d l_target_cam;
    static Eigen::Matrix4d l_target_F0;
    static Eigen::Matrix4d H_icam_L;
    static Eigen::VectorXd l_q;
    static Eigen::VectorXd xx;
    static Eigen::VectorXd xx5;
    static Eigen::MatrixXd Q;

    static double costQ;
    static double cost;

    static Kinematic::GeometricModel model;

    static Eigen::Vector4d zVecConst;

    // to know if the model will uses real or approximate z
    static bool zConst;

    /// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///

    public:
    // Constructor moving the boolean value into the class
    explicit Model(bool&& zConst);
    // Constructor by copy
    explicit Model(bool& zConst);

    void setup_matrixes(int nbFeatures, int nbPredictions, int nbJoints);
    ~Model();
  };
}

#endif // MODEL_CLASS_H