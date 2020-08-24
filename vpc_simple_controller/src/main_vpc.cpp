/*
 * File: main_vpc.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Wednesday, 3rd June 2020 12:59:19 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Thursday, 13th August 2020 6:41:31 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include <ros/ros.h>

#include <vpc_simple_controler/wrapperController.h> 


int main(int argc, char** argv){

    // Init the ROS node
    ros::init(argc, argv, "robot_vpc");
    ros::NodeHandle nh_;
    Wrapper::WrapperController wrapper(&nh_);
    ros::Rate loop_rate(.5);

    std::cout << "Debut du run" << std::endl;

    while (ros::ok()){// for(int idx = 0; idx < 6; idx++
        // loop_rate.reset();
        ros::spinOnce();
        wrapper.controlLoop();
        //Wait for the end of the loop
        loop_rate.sleep();
        std::cout << "\nAfter sleep\n";
        if(!wrapper.isVS_ON){ //if end of the VPC shutdown 
            // ros::spinOnce();
            ros::shutdown();
            std::cout << "Fin du run, wait for ctrl-C key to close" << std::endl;
        }
    }
    return 0; 
}
