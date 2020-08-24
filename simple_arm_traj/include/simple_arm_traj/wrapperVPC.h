/*
 * File: wrapperVPC.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Saturday, 2nd May 2020 12:29:44 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Tuesday, 12th May 2020 11:13:00 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#ifndef WRAPPERVPC_CLASS_H
#define WRAPPERVPC_CLASS_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include "/home/adurandp/ros/tiago_public_ws/devel/include/adp_vision/visualFeatures.h"
//#include "adp_control/controllerVPC.h"

// test de eigen 
#include <Eigen/Core>
#include <Eigen/Dense>
// test de nlopt
#include <Nlopt/nlopt.h>


class wrapperVPC {
    
    public:
        wrapperVPC(ros::NodeHandle nh);

        // void callbackVF(const adp_vision::visualFeatures::ConstPtr&);

        // void callbackControl(const ros::TimerEvent& e);

        // void controlLoop();

        bool get_VS_on();

    private:
        ros::Timer timerControl;

        //controllerVPC vpc;
        ros::Subscriber image_sub;

        float* bufferVF; // To convert from msg to vpc class
        float* bufferZ;

        ros::Publisher pub_base;

        // Messagess to be published
        geometry_msgs::Twist msg_twist;

        float ts;
        double thetaHead;
        float* command;

        bool imAvailable, VS_on;
};
#endif // ifndef WRAPPERVPC_CLASS_H