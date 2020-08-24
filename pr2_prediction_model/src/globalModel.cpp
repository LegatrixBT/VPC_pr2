/*
 * File: globalModel.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Tuesday, 23rd June 2020 2:38:39 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Monday, 24th August 2020 5:43:07 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include <pr2_prediction_model/globalModel.h>


GlobalModel::~GlobalModel(){
    delete my_f_data;
    std::cout << "del de GM\n";
}

double GlobalModel::myfunc(const std::vector<double> &x, std::vector<double> &grad, void *func_data){
    Timer timer;
    my_func_data* d = reinterpret_cast<my_func_data*> (func_data);
    // gather the information back  --> References faster ??? to try ?
    bool& r_arm         = d->r_arm;
    int& nbPredictions  = d->nbPredictions;
    int& nbJoints       = d->nbJoints;
    int& nbFeatures     = d->nbFeatures;
    double& ts          = d->ts;
    float& focal        = d->focal;
    
    bool& z_real                = d->z_real;
    float& l_zConst             = d->l_zConst;
    Eigen::VectorXd& l_VF       = d->l_VF;
    Eigen::VectorXd& l_VF_ref   = d->l_VF_ref;
    Eigen::VectorXd& l_VF_depth = d->l_VF_depth; 
    Eigen::VectorXd& l_q_state  = d->l_q_state; 
    
    ///////////////////////// corps de la fonction ////////////////////////////
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    // std::cout << "\n NEW RUN \n\n";
    // std::cout << "l_VF" << l_VF.transpose().format(CleanFmt) << "\n";
    // //// /// // / Init / // /// ////
        
    S_ref_l = l_VF_ref;
    for(auto i =0; i<x.size();i++){xx[i]=x[i];};
    // std::cout << "xx=\n" << xx.transpose().format(CleanFmt) <<"\n";
    zVecConst = {l_zConst, l_zConst, l_zConst, l_zConst};
    Eigen::Vector4d a;
    a << focal/l_zConst, focal/l_zConst, 1, 1;
    Hi2cam = Eigen::MatrixXd::Zero(16,16);
    Hi2cam.diagonal() << a,a,a,a;
    // if(zConst) std::cout << "toto";
    l_q = l_q_state.head(nbJoints);;
    // std::cout << "\n l_q=" << l_q.transpose().format(CleanFmt) << "\n";
    Sm_l << l_VF(0), l_VF(1), l_VF_depth(0), 1,
            l_VF(2), l_VF(3), l_VF_depth(1), 1,
            l_VF(4), l_VF(5), l_VF_depth(2), 1,
            l_VF(6), l_VF(7), l_VF_depth(3), 1;
    // std::cout << "\n Sm_l=" << Sm_l.transpose() << "\n";
    // std::cout << "\n Hi2cam=\n" << Hi2cam << "\n\n";

    model.evaluate5Joints(l_q, false);
    // // -------------------------------------------- // //

    // Boucle sur le nb de prediction 
    for(auto idxPred = 0; idxPred < nbPredictions; idxPred++){ 
  
        for(auto i=0; i < nbFeatures; i++){
            // projectionMatrix_zConst(H_icam_L, focal, l_zConst);    // std::cout << "x [";
            H_icam_L = Hi2cam.block(i*4,i*4,4,4);
            l_target_cam.col(i) = H_icam_L.colPivHouseholderQr().solve(Sm_l.segment(i*4,4));
        }
        l_target_F0 =  model.TBC()*l_target_cam;
        // std::cout << "\n l_target_F0=\n" << l_target_F0 << "\n";
        // std::cout << "\n l_target_cam=\n" << l_target_cam << "\n\n";
  
        // xx5 = xx.head(nbJoints); // normal
        xx5 = xx.segment<5>(5*idxPred); // suivant le deplacement dans le vecteur
        // std::cout << "\n xx5=\n" << xx5.transpose().format(CleanFmt) << "\n\n";
        l_q = l_q + xx5 * ts; 
        model.evaluate5Joints(l_q,false);  // evaluate the geoModel with the 5 joints

        //Predictive Global Model --------------
        //Compute target_cam(k+1) from target_base(k)

        l_target_cam = model.TBC().colPivHouseholderQr().solve(l_target_F0);
        // std::cout << "\n l_target_cam=\n" << l_target_cam << "\n";

        // Update z_ref if zReal
        if(z_real){
            l_VF_depth = l_target_cam.col(2); // affect the z just estimated // hat{Z} = Z
        };

        for(auto i=0; i < nbFeatures; i++){
            H_icam_L = Hi2cam.block(i*4,i*4,4,4);
            S_temp_l.col(i) = H_icam_L * l_target_cam.col(i);
        }
        // std::cout << "\n S_temp_l=\n" << S_temp_l << "\n";

        // ----------> temp_L_VF doit prendre les deux premiere valeur de chaque colonne 
        temp_l_VF = S_temp_l.topRows<2>();
        S_eval = Eigen::Map<Eigen::VectorXd>(temp_l_VF.data(), l_VF.size());

        // Calculation of the squared error on the visual features
        Q = Eigen::MatrixXd::Identity(2*nbFeatures,2*nbFeatures);
        
        // double lambda = 1.25;
        // costQ = lambda/(lambda *(idxPred+1)); // to be able to put an exponential decrease etc... 
        costQ = 1;
        Q = Q *costQ;
                
        // Cost calculation 
        cost = (S_ref_l - S_eval).transpose()*Q*(S_ref_l - S_eval);
        // std::cout << "\n S_eval=\n" << S_eval.transpose().format(CleanFmt) << "\n";
        // std::cout << "\n S_ref_l=\n" << S_ref_l.transpose().format(CleanFmt) << "\n";
        // std::cout << "\n cost=" << cost << "\n";

        // historic of the costs through optimizations 
        C[idxPred] = cost;
    }

    // std::cout << "\n S_temp_l=\n" << S_temp_l << "\n";
    // std::cout << "\nExit fct with:\nl_VF=" << l_VF.transpose().format(CleanFmt) << "\n";
    // std::cout << "\nl_VF_temp//S_eval=" << S_eval.transpose().format(CleanFmt) << "\n";
    // std::cout << "xx=\n" << xx.transpose().format(CleanFmt) <<"\n";
    // std::cout << "Cout=" << C.transpose().format(CleanFmt) << "\nSomme=" << C.sum() << "\nSqrt=" << sqrt(C[0]) << "\n";
    // double minf = sqrt(C.sum());
    // double minf = sqrt(C[0]);
    double minf = sqrt(C.sum());

    // std::cout << "sqrt CC=" << sqrt(CC.sum()) << "\n";
    // std::cout << "In fct minf=" << minf <<"\n";
    return minf;
}


