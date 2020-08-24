/*
 * File: VPCController.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Wednesday, 3rd June 2020 12:37:41 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Monday, 24th August 2020 10:35:30 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include <vpc_simple_controler/VPCController.h>


    VPCCtrl::VPCController::VPCController(){

        ts = .5;

        nbPredictions = 50 ;  //5 et ts de 0.5 ou 1 
        nbFeatures = 4;

        focal  = 1.f; // 1.0; // Not a normalized focal --> to be change

        // Setting up the robot vectors
        l_state.resize(NB_JOINTS);
        l_q_state.resize(NB_JOINTS);
        r_state.resize(NB_JOINTS);
        l_command.resize(NB_JOINTS);
        r_command.resize(NB_JOINTS);
        // Initialize them 
        l_command << 0, 0, 0, 0, 0;
        r_command << 0, 0, 0, 0, 0;

        // Setting up the VPC vectors
        l_VF.resize(2*nbFeatures);
        l_VF_ref.resize(2*nbFeatures);
        l_VF_depth.resize(nbFeatures);

        r_VF.resize(2*nbFeatures);
        r_VF_ref.resize(2*nbFeatures);
        r_VF_depth.resize(nbFeatures);

        ///////////TO REMOVE ///////////
        l_VF.setZero();
        l_VF_depth.setZero();

        ////////////////////////////////

        // for the depth gestion 
        l_zReal  = false;
        l_zConst = .5; // setting up to 90cm

        l_zReal  = false;
        l_zConst = .5; // setting up to 90cm

        // if we have to warm start

        std_l_x_prev = new std::vector<double>(NB_JOINTS*nbPredictions,0);
        std_r_x_prev = new std::vector<double>(NB_JOINTS*nbPredictions,0);

        // for(int xIdx = 0; xIdx < NB_JOINTS*nbPredictions; xIdx++){
        //     std_l_x_prev->at(xIdx) = 0.0;
        //     std_r_x_prev->at(xIdx) = 0.0;
        // }

    }

    //----------------------------------------------------------------------------//

    //! Free all the memory allocated for the solver
    VPCCtrl::VPCController::~VPCController(){

        // ---------- Free the solver variables ---------- //

        // Free the states
        delete std_l_x;
        delete std_l_x_prev;
        
        delete std_r_x;
        delete std_r_x_prev;

        delete opt; // may not be necessary 
        delete gModel;
    }

    //----------------------------------------------------------------------------//
        double myfuncVPC(const std::vector<double> &x, std::vector<double> &grad, void *func_data);

    const void VPCCtrl::VPCController::setupSolver(){

        // Creating an instance of the solver object
        opt = new nlopt::opt(nlopt::LN_PRAXIS, NB_JOINTS*nbPredictions);

        // Define the upper and lower bounds for the solver
        v_lb.resize(NB_JOINTS*nbPredictions);
        v_ub.resize(NB_JOINTS*nbPredictions);
        // @TODO --> affiner les limites pour être un poil plus réalistes 
        for(int i = 0; i < nbPredictions; i++){
            v_lb[NB_JOINTS*i    ]   = -0.1; // -3.14;
            v_lb[NB_JOINTS*i + 1]   = -0.1; // -3.14;
            v_lb[NB_JOINTS*i + 2]   = -0.1; // -3.14;
            v_lb[NB_JOINTS*i + 3]   = -0.1; // -3.14;
            v_lb[NB_JOINTS*i + 4]   = -0.1; // -3.14;

            v_ub[NB_JOINTS*i    ]   =  0.1; // 3.14;
            v_ub[NB_JOINTS*i + 1]   =  0.1; // 3.14;
            v_ub[NB_JOINTS*i + 2]   =  0.1; // 3.14;
            v_ub[NB_JOINTS*i + 3]   =  0.1; // 3.14;
            v_ub[NB_JOINTS*i + 4]   =  0.1; // 3.14;
        }
        opt->set_lower_bounds(v_lb);
        opt->set_upper_bounds(v_ub);

        // states to be optimized
        std_l_x = new std::vector<double>(NB_JOINTS * nbPredictions);
        std_r_x = new std::vector<double>(NB_JOINTS * nbPredictions);

        //Preparing the structure for the solver
        gModel = new GlobalModel(l_zConst);
        bool r_arm = false;
        nbJoints = (int)NB_JOINTS;

        // std::cout << "&l_VF=" << &l_VF << "\n";
        // std::cout << "l_VF=" << l_VF.transpose() << "\n";
        gModel->setup_matrixes(nbFeatures, nbPredictions, NB_JOINTS);
        gModel->my_f_data = new GlobalModel::my_func_data(&r_arm, &nbPredictions, &nbFeatures, &nbJoints, &ts, &focal, &l_zConst, &l_zReal,
                                                          &l_VF, &l_VF_ref, &l_VF_depth, &l_q_state);
        // std::cout << "my_f_data &l_VF_ref=" << &gModel->my_f_data->l_VF_ref<<"\nmy_f_data fin\n";
        // std::cout << "my_f_data l_VF_ref=" << gModel->my_f_data->l_VF_ref.transpose()<<"\nmy_f_data fin\n";
        
        //! Link the objective function
        opt->set_min_objective(gModel->myfunc, gModel->my_f_data);

        // opt->set_maxtime(20); // max time before timeout
        // opt->set_maxeval(2000);
        opt->set_maxtime(5);

        // Choose the stopping criteria for the optimization


        VPC_stop = .035;
        stop_val = 0.001;
        opt->set_stopval(stop_val);
        // opt->set_ftol_rel(stop_val);
        // opt->set_ftol_abs(stop_val);

        ///// INITIAL GUESS
        // std_l_x_prev->at(0) = -0.10;
        // std_l_x_prev->at(1) =  0.01;
        // std_l_x_prev->at(2) = -0.1;
        // std_l_x_prev->at(3) = -0.07;
        // std_l_x_prev->at(4) = -0.1;

        // std_l_x_prev->at(5) = -0.05;
        // std_l_x_prev->at(6) = -0.05;
    }

    // Set the state for an arm
    const void VPCCtrl::VPCController::set_state(const bool r_arm, std::vector<double>& state){
        r_arm ? r_state = state : l_state = state;
    }

    //----------------------------------------------------------------------------//
    // Set the reference visual features
    // @TODO, by ref and do an std::move ? ?? 
    const void VPCCtrl::VPCController::setVF_ref(const bool r_arm, double x0, double y0, double x1, double y1,
                                           double x2, double y2, double x3, double y3){
        if(r_arm){
            r_VF_ref[0] = x0;
            r_VF_ref[1] = y0;
            r_VF_ref[2] = x1;
            r_VF_ref[3] = y1;
            r_VF_ref[4] = x2;
            r_VF_ref[5] = y2;
            r_VF_ref[6] = x3;
            r_VF_ref[7] = y3;
        }else{
            l_VF_ref[0] = x0;
            l_VF_ref[1] = y0;
            l_VF_ref[2] = x1;
            l_VF_ref[3] = y1;
            l_VF_ref[4] = x2;
            l_VF_ref[5] = y2;
            l_VF_ref[6] = x3;
            l_VF_ref[7] = y3;
            std::cout << "VF* =["<< l_VF_ref.transpose()<<"]\n";
        }
    }

    const void VPCCtrl::VPCController::setVF_as_VF_ref(const bool r_arm){
        if(r_arm){
            r_VF_ref = r_VF;
        }else{
            l_VF_ref = l_VF;
            std::cout << "VF* =["<< l_VF_ref.transpose()<<"]\n";
        }
    }

    //----------------------------------------------------------------------------//
    // Get the sampling time
    float VPCCtrl::VPCController::getTs() const{
        return ts;
    }

    // Get the number of features
    int VPCCtrl::VPCController::getNbFeatures() const{
        return nbFeatures;
    }

    const void  VPCCtrl::VPCController::updateVF(const bool r_arm, float* vf){ // TODO test with a std::move instead of affectation if vf not need outside
        
        if(r_arm){
            for(int i = 0; i <r_VF.size(); i++){
                r_VF[i] = vf[i];
            }
        }else{
            for(int i = 0; i <l_VF.size(); i++){
                l_VF[i] = vf[i];
            }
        }
        // std::cout << "VF UPDATED WITH["<< l_VF.transpose()<<"]\n";
    }

    //----------------------------------------------------------------------------//
    const void  VPCCtrl::VPCController::updateDepthVF(const bool r_arm, float* z){
        
        if(!l_zReal || !r_zReal){ 
            for(int i = 0; i < r_VF_depth.size(); i++){
                r_VF_depth[i] = r_zConst;
                l_VF_depth[i] = l_zConst;
            }
            return;
        }
        if(r_arm){
            for(int i = 0; i < r_VF_depth.size(); i++){
                r_VF_depth[i] = z[i];
            }
        }else{
            for(int i = 0; i < l_VF_depth.size(); i++){
                l_VF_depth[i] = z[i];
            }
        }
        // std::cout << "Z_VF ["<< l_VF_depth.transpose() << "]\n";
    }

    // ----------------------------------------------------------------------------//
    // Return true if the task is considered achieved, 
    // if the sum of the squared error is under the ERROR_VAL_STOP_VPC value
    bool VPCCtrl::VPCController::taskAchieved(const bool r_arm) const{
        
        double dist;
        if(r_arm){
                // sum of the squared error 
                dist = (r_VF - r_VF_ref).transpose() * (r_VF - r_VF_ref);
        }else{{
                // sum of the squared error 
                dist = (l_VF - l_VF_ref).transpose() * (l_VF - l_VF_ref);
            }
        }
        std::cout << "Task? " << (sqrt(dist) < VPC_stop ? true : false) << " achieved [" << sqrt(dist) << "]" <<std::endl;

        return sqrt(dist) < VPC_stop ? true : false;
    }

   
    void  VPCCtrl::VPCController::vpc(const bool r_arm, Eigen::VectorXd* q){

        Timer timer;
        for(int predIdx = 0; predIdx < nbPredictions; predIdx++){ //reload the previous prediction
            std_l_x->at(NB_JOINTS*predIdx)     = std_l_x_prev->at(NB_JOINTS*predIdx);
            std_l_x->at(NB_JOINTS*predIdx + 1) = std_l_x_prev->at(NB_JOINTS*predIdx + 1);
            std_l_x->at(NB_JOINTS*predIdx + 2) = std_l_x_prev->at(NB_JOINTS*predIdx + 2);
            std_l_x->at(NB_JOINTS*predIdx + 3) = std_l_x_prev->at(NB_JOINTS*predIdx + 3);
            std_l_x->at(NB_JOINTS*predIdx + 4) = std_l_x_prev->at(NB_JOINTS*predIdx + 4);
        }

        // gather the q of the arms 
        l_q_state = *q;
        // std::cout << "vpc &Q=" << &l_q_state << "\n";
        try{
            std::cout << "algo [" << opt->get_algorithm_name() << "]\n";
            std::cout << "ftol_rel [" << opt->get_ftol_rel() << "] ftol_abs [" << opt->get_ftol_abs() << "] stop val[" << opt->get_stopval() << "\n";
 
            std::vector<double> x(NB_JOINTS*nbPredictions);

            // std::cout << "copie de std_lx in x\n[";
            for(int i=0; i<NB_JOINTS*nbPredictions; i++){
                x[i] = std_l_x->at(i);
                // std::cout << x[i] <<", ";
            }
            // std::cout << "]\n";
            timer.~Timer();
            nlopt::result ibvs_res = opt->optimize(x, minf);
            timer.~Timer();
            std::cout << "\nCODE RETOUR opti =" << ibvs_res << "\n";
            // if(ibvs_res == 2 ){
            //     stop_val -= stop_val*.05;
            //     opt->set_stopval(stop_val);      
            //     std::cout << "new objectif =" << stop_val <<"\n";
            // }
            // if(ibvs_res == 3){
            //     stop_val -= stop_val*.05;
            //     opt->set_ftol_rel(stop_val);      
            //     std::cout << "new objectif =" << stop_val <<"\n";
            // }

            std::cout << "VF[" << l_VF.transpose() << "]\n";
            std::cout << "VF_ref[" << l_VF_ref.transpose() << "]\n\n";

            // std::cout << "last opti val =" << opt->last_optimum_value() <<"\n";
            // std::cout << "x after optimization &x=" << &x << "\n[";
            for(int i=0; i<NB_JOINTS*nbPredictions; i++){
                // std::cout << x[i] <<", ";
                std_l_x->at(i) = x[i];
            }
            std::cout << "]\n";

            if(ibvs_res < 0){
                std::cout << "nlopt failed ... again :|\n";
                l_command << 0,0,0,0,0; // first input of the command set as no mvt
            }else
            {
                std::cout << "nlopt success !\n";
                l_command << std_l_x->at(0), std_l_x->at(1), std_l_x->at(2) ,std_l_x->at(3), std_l_x->at(4);
                
                //---> les consommer en déplacant  toutes les autres valeurs de NBjoints vers la gauche et 0 padding 
                for(auto i=0; i< std_l_x->size()-NB_JOINTS; i++){
                    std_l_x->at(i)=std_l_x->at(i+NB_JOINTS);
                }
                for(auto i=std_l_x->size()-NB_JOINTS; i<std_l_x->size(); i++){std_l_x->at(i)=0;};

                // saving the optimization results for next iteration
                std::cout << "new l_command ="<< l_command.transpose() <<"\n";
                for(int xIdx = 0; xIdx < NB_JOINTS*nbPredictions; xIdx++){
                    std_l_x_prev->at(xIdx) = std_l_x->at(xIdx);
                }
            }
            std::cout << "\nMINF= " << minf << "\n\n";
        }catch(std::exception &e) {
            std::cout << "nlopt failed: " << e.what() << std::endl;
        }
    }