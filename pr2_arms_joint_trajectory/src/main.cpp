/*
 * File: main.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Friday, 5th June 2020 6:37:00 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Thursday, 11th June 2020 5:55:00 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */







#include <pr2_arms_joint_trajectory/arms_joint_traj.h>
// #include "../include/pr2_arms_joint_trajectory/arms_joint_traj.h"

int main(int argc, char** argv){

    // Init the ROS node
    ros::init(argc, argv, "robot_driver");


    ros::NodeHandle nh;
    Arms_joint_traj::RobotArms arms(&nh);


    if(argc == 2 && argv[1] == std::string("reset")){ //argv[1]=="reset"){
        std::cout << "pr2 is reseting arms positions :\n\n" <<  arms ; 

        // ~= Init du robot 
        std::vector<double> q = {0, 0.1, 0, -0.285, 0, -0.4, 0};

        arms.set_arm_goal(q,true);
        arms.set_arm_goal(q,false);         
        arms.startBothTrajectory(0);

        while(arms.getLState().isDone() && arms.getRState().isDone() && ros::ok()){
            usleep(50000);
        }
        std::cout << "done !\n"; 

        //std::array<double,7> test = __is_array_unknown_bounds q;
        //std::cout << "Val de test : " << test[0] << std::endl;
        return 0;
    }

    // Start the trajectory
    std::cout << "Debut du run" << std::endl;
    arms.startBothTrajectory(arms.arm_joint_movement_user_verif(true), arms.arm_joint_movement_user_verif(false), 0);

    while( arms.getLState().isDone() && arms.getRState().isDone() && ros::ok()){
        usleep(50000);
    }

    std::cout << "Fin du run" << std::endl;
    return 0;
}