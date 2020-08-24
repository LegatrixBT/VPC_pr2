/*
 * File: wrapperController.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Thursday, 4th June 2020 1:49:45 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Monday, 24th August 2020 5:41:10 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include <vpc_simple_controler/wrapperController.h>

#include<memory>

Wrapper::WrapperController::WrapperController(ros::NodeHandle *nh){
    
    
    // Allocate some memory for the buffers 
    int nbFeatures = vpc.getNbFeatures();
    l_bufferVF = new float[2*nbFeatures];
    l_depthVF  = new float[nbFeatures];

    r_bufferVF = new float[2*nbFeatures];
    r_depthVF  = new float[nbFeatures];

    l_command    = new double[NB_JOINTS];
    r_command    = new double[NB_JOINTS];

    // allocate the memory for the arms object 
    arms = new Arms_joint_traj::RobotArms(nh);
    
    // Get the sampling time 
    ts = vpc.getTs();

    // Setup the boolean to be sure that an image is available
    isLAvailable = false;
    isRAvailable = false;
    isVS_ON      = true;

    // Subscribe to the visual features topics if possible 
    try{
        // Subscribe in a diferent thread for each or 1 thread for both according to how async spinner is created
        l_ops = ros::SubscribeOptions::create<pr2_forearm_vison::visualFeatures>(
            "l_visual_features",1,
            boost::bind(&Wrapper::WrapperController::callbackVF_l, this, _1),
            ros::VoidPtr(), &VF_queue);
        l_ops.transport_hints = ros::TransportHints();
        l_ops.allow_concurrent_callbacks = true;
        img_left_sub = nh->subscribe(l_ops);
        r_ops = ros::SubscribeOptions::create<pr2_forearm_vison::visualFeatures>(
            "r_visual_features",1,
            boost::bind(&Wrapper::WrapperController::callbackVF_r, this, _1),
            ros::VoidPtr(), &VF_queue);
        r_ops.transport_hints = ros::TransportHints();
        r_ops.allow_concurrent_callbacks = true;
        img_right_sub = nh->subscribe(r_ops);
        // img_left_sub  = nh->subscribe("l_visual_features", 1, &Wrapper::WrapperController::callbackVF_l, this);
        // img_right_sub = nh->subscribe("r_visual_features", 1, &Wrapper::WrapperController::callbackVF_r, this);
        VF_spinner = new ros::AsyncSpinner(1,&VF_queue);
        VF_spinner->start();
        std::cout << " subcribers are ready's \n";
    }
    catch(ros::Exception e){
        ROS_WARN("Error occured while subscribing to one of the topics l/r_visual_features \n[%s]", e.what());
    }

    // Set the visual feature as a reference 

    vpc.setVF_ref(false, -0.116, -0.180, 0.170, -0.180, 0.170, 0.1, -0.116, 0.1);
    // vpc.setVF_ref(false, -116, -180, 0170, -180, 0170, 0100, -0116, 0100);
    vpc.setVF_ref(true, 0,0,0,0,0,0,0,0); /// A définir !
    // std::cout << "wrapp l_VF_ref=" << vpc.l_VF_ref.transpose()<<"\n";

    // Then setup the solver (bounds and algo type)
    vpc.setupSolver();

    // Move the arms to init position before doing the vpc
    // position of the arms at bringup

    std::vector<double> arms_bringup_pos = {0, 0.010, 0.002, -0.294, 0, -0.413, 0};
    std::ostringstream vts, vtss;  // just for have a clean stdout of the vectors 
    std::copy(arms_bringup_pos.begin(), arms_bringup_pos.end()-1, std::ostream_iterator<double>(vts, ", ")); 
    vts << arms_bringup_pos.back(); 
    std::cout << "q* positions [" <<  vts.str() << "]\n";
    // position of the arms before the vs
    // std::vector<double> arms_init_vpc_pos = {0, 0.2, 0.002, -0.3, 0, -0.4, 0};
    
    // easy
    std::vector<double> arms_init_vpc_pos = {0.025, 0.0, 0.025, -0.2, 0.1, -0.2, 0}; // eloigné ici 
    // plus difficile
    // std::vector<double> arms_init_vpc_pos = {0.03, 0.32,0.035, -0.69, -0.054, -.2, 0};
    // bras vers le bas 
    // std::vector<double> arms_init_vpc_pos = {0, .85, 0, -1.15, .002, -.01, 0};
    // bras vers le  tres bas + rotation
    // std::vector<double> arms_init_vpc_pos = {0, 1.3, 0, -1.4, .32, -.01, 0};
    // hard perte de visualisation --> ajout de contraintes de non sortie etc...
    // std::vector<double> arms_init_vpc_pos = {1.0, 0.132, 0.31, -1.297, 1.18, -0.1, 0};
    // Far away but just flex the arm loose vision too
    // std::vector<double> arms_init_vpc_pos = {.01, .75, .15, -.97, .001, -.01, 0};
    
    // std::vector<double> arms_init_vpc_pos = {-0.05, -0.05, 0, 0, -0.26, -0.4, 0}; // eloigné ici 
    std::copy(arms_init_vpc_pos.begin(), arms_init_vpc_pos.end()-1, std::ostream_iterator<double>(vtss, ", ")); 
    vtss << arms_init_vpc_pos.back(); 
    std::cout << "q positions @start[" <<  vtss.str() << "]\n";
    std::cout << " nbPred=" << vpc.nbPredictions << "\nnbJoints=" << NB_JOINTS << "\n";

    arms->set_arm_goal(arms_init_vpc_pos, false);
    arms->set_arm_goal(arms_init_vpc_pos, true);
    // arms->set_arm_goal(arms_bringup_pos, false);
    // arms->set_arm_goal(arms_bringup_pos, true);
    
    arms->startBothTrajectory(0);
    while(arms->getLState().isDone() && arms->getRState().isDone() && ros::ok()){
        usleep(50000);
    }
    // wait 3 sec to be sure that the arms are at the new position
    sleep(1); 

    // Setup the commands (q_dot) for the robot 
    std::vector<double> r_init_state = {0,0,0,0,0};
    std::vector<double> l_init_state = {0,0,0,0,0};

    vpc.set_state(true, r_init_state);
    vpc.set_state(false, l_init_state);
}

Wrapper::WrapperController::~WrapperController(){

    delete[] l_bufferVF;
    delete[] l_depthVF;
    delete[] r_bufferVF;
    delete[] r_depthVF;
    delete[] l_command;
    delete[] r_command;
    delete VF_spinner;
    delete arms;

}

//! Copy the visuals features into the buffers then update them in the VPC controller
void Wrapper::WrapperController::callbackVF_l(const pr2_forearm_vison::visualFeatures::ConstPtr& msg){

    // Copy of the msgs into the buffers
    l_bufferVF[0] = msg->x1;
    l_bufferVF[1] = msg->y1;
    l_bufferVF[2] = msg->x2;
    l_bufferVF[3] = msg->y2;
    l_bufferVF[4] = msg->x3;
    l_bufferVF[5] = msg->y3;
    l_bufferVF[6] = msg->x4;
    l_bufferVF[7] = msg->y4;

    l_depthVF[0] = msg->z1;
    l_depthVF[1] = msg->z2;
    l_depthVF[2] = msg->z3;
    l_depthVF[3] = msg->z4;

    // Update the features into the VPC controller 
    vpc.updateVF(false, l_bufferVF); // Change for a move to save time ? Try to time the difference 
    vpc.updateDepthVF(false, l_depthVF);

    // Set the bool isAvailable to true after fill all the datas
    isLAvailable = true; 

}

//! Copy the visuals features into the buffers then update them in the VPC controller
void Wrapper::WrapperController::callbackVF_r(const pr2_forearm_vison::visualFeatures::ConstPtr& msg){
    
    // Copy of the msgs into the buffers
    r_bufferVF[0] = msg->x1;
    r_bufferVF[1] = msg->y1;
    r_bufferVF[2] = msg->x2;
    r_bufferVF[3] = msg->y2;
    r_bufferVF[4] = msg->x3;
    r_bufferVF[5] = msg->y3;
    r_bufferVF[6] = msg->x4;
    r_bufferVF[7] = msg->y4;

    r_depthVF[0] = msg->z1;
    r_depthVF[1] = msg->z2;
    r_depthVF[2] = msg->z3;
    r_depthVF[3] = msg->z4;

    // Update the features into the VPC controller 
    vpc.updateVF(true, r_bufferVF); // A changer pour un move ??? dans la classe Controller VPC !  
    vpc.updateDepthVF(true, r_depthVF);

    // Set the bool isAvailable to true after fill all the datas
    isRAvailable = true;
}

float Wrapper::WrapperController::getTs() const{
    return vpc.getTs();
}

void Wrapper::WrapperController::controlLoop(){

    std::cout << "Entering Ctrl_Loop\n";
    bool r_arm = false; 
    bool position = false;
    bool velocity = true;
    
    Eigen::VectorXd current_lq, current_rq;
    Eigen::VectorXd q_dot(7);
    std::vector<double> next_std_lq(7);
    std::vector<double> next_std_lqdot(7,1e-5);
    Eigen::VectorXd next_lq(7);
    
    if(isLAvailable){
        if(!vpc.taskAchieved(r_arm) && isVS_ON){
            std::cout << "Go \n";
            // get the q of the arms 
            arms->get_arms_joints_info(true, current_lq, current_rq);
            vpc.vpc(r_arm, &current_lq);
            // std::cout << "wrap &Q=" << &lq << "\n"; // same object in globalModel than here for lq -- l_q_state
            
            //Apply the commands
            //  Gather the curren q 
            // arms->get_arms_joints_info(true, current_lq, current_rq);
            
            q_dot[0] = vpc.l_command[0]; //Eigen::VectorXd::Map(vpc.std_l_x->data(), NB_JOINTS+2);
            q_dot[1] = vpc.l_command[1];
            q_dot[2] = vpc.l_command[2];
            q_dot[3] = vpc.l_command[3];
            q_dot[4] = vpc.l_command[4];
            q_dot[5]=0;  q_dot[6]=0; // no mvt for q 5 and 6 
            // std::cout << "curr l_q=\n" <<  current_lq.transpose()<< "\n";
            // std::cout << "q_dot*ts =\n" <<  (q_dot * vpc.getTs()).transpose()<< "\n";
            
            next_lq = current_lq + q_dot * vpc.getTs();
            std::cout << "next l_q=\n" << next_lq.transpose()<< "\n";

            // map in stdVector to publish the goal
            Eigen::VectorXd::Map(&next_std_lq[0], next_lq.size()) = next_lq;

            // next_std_lq(next_lq.data(), next_lq.data() + next_lq.rows() * next_lq.cols());
            // set and apply the mvt
            // arms->set_arm_goal(next_std_lq, r_arm);
            // -------−> set the goal in position AND velocities in the arm controller !
            arms->set_arm_goal(position, next_std_lq, r_arm);
            // try with velocities fixed 
            // std::vector<double> next_std_lqdot(q_dot.data(), q_dot.data() + q_dot.rows() * q_dot.cols());

            // std::vector<double> accelerations(5,1); BUGGED ???? 
            // arms->set_arm_goal_acceleration(accelerations, r_arm);

            arms->set_arm_goal(velocity, next_std_lqdot, r_arm);

            arms->startBothTrajectory(0);
            while(arms->getLState().isDone() && ros::ok()){
                usleep(50000);
            }
            sleep(.5);
            
            // and update the vpc/wrapper
            current_lq = next_lq;
            // wrapper.vpc.l_state = x  --> x devient state
            // std::move(next_std_lq.begin(), next_std_lq.begin() + NB_JOINTS, vpc.l_q_state.begin());
            
        }else{std::cout << "stop\n"; isVS_ON = false;}
    }
    
}
