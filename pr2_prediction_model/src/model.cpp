/*
 * File: model.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Tuesday, 23rd June 2020 1:42:06 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Friday, 14th August 2020 10:33:47 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include <pr2_prediction_model/model.h>
#include <iostream>

Prediction::Model::Model(bool&& zConst){
    std::cout << "Constructeur Model (move)\n"; 
    this->zConst = zConst;
    zConst = 0;
}

Prediction::Model::Model(bool& zConst){
    std::cout << "Constructeur Model (copy)\n"; 
    this->zConst = zConst;
}



Prediction::Model::~Model(){
    std::cout << "~Destructeur Model\n"; 

}



bool Prediction::Model::zConst;


Eigen::VectorXd Prediction::Model::C;
Eigen::VectorXd Prediction::Model::S_l;
Eigen::VectorXd Prediction::Model::S_ref_l;
Eigen::VectorXd Prediction::Model::Sm_l;
Eigen::Matrix4d Prediction::Model::S_temp_l;
Eigen::MatrixXd Prediction::Model::temp_l_VF;
Eigen::VectorXd Prediction::Model::S_eval;
Eigen::MatrixXd Prediction::Model::Hi2cam;

Eigen::Matrix4d Prediction::Model::l_target_cam;
Eigen::Matrix4d Prediction::Model::l_target_F0;
Eigen::Matrix4d Prediction::Model::H_icam_L;
Eigen::VectorXd Prediction::Model::l_q;
Eigen::VectorXd Prediction::Model::xx;
Eigen::VectorXd Prediction::Model::xx5;
Eigen::MatrixXd Prediction::Model::Q;

double Prediction::Model::costQ;
double Prediction::Model::cost;

Kinematic::GeometricModel Prediction::Model::model;

Eigen::Vector4d Prediction::Model::zVecConst;

void Prediction::Model::setup_matrixes(int nbFeatures, int nbPredictions, int nbJoints){
    std::cout << "\nSetup the matrixes and vect length!\n"; 
    
    xx.resize(nbJoints*nbPredictions);
    xx5.resize(5);
    C.resize(nbPredictions);

    S_l.resize(16);
    S_ref_l.resize(16);
    Sm_l.resize(16); 
    Hi2cam.resize(16,16);
    Sm_l.resize(16);
    Q.resize(2*nbFeatures,2*nbFeatures);


}
