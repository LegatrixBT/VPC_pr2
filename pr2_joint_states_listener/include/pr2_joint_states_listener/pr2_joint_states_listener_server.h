/*
 * File: pr2_joint_states_listener_server.h
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Saturday, 16th May 2020 11:02:08 am
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Thursday, 11th June 2020 7:18:53 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */







#ifndef PR2_JOINT_STATES_LISTENER_SERVER_CLASS_H
#define PR2_JOINT_STATES_LISTENER_SERVER_CLASS_H

/// ---------------------- HEADERS ------------------------- ///


#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <thread>
#include <mutex>

// Include of the custom server header for listen the joint_state topic 
#include "pr2_joint_states_listener/pr2_joint_states_listener.h"

/// ----------------------- DEFINES ------------------------- ///

// Define the size of the queue buffer before message are throw 
#define SUB_QUEUE_SIZE 1

/// ------------------------ CLASS -------------------------- ///

class LastestJointStates {

    private:
    
    ros::Subscriber sub;
    ros::ServiceServer srv;

    // @TODO Fix the vector size to reduce execution time 
    std::vector<std::string> names; 
    std::vector<double> position;
    std::vector<double> effort;
    std::vector<double> velocity;

    // Current values of a joint 
    double current_position;
    double current_effort;
    double current_velocity;
    unsigned int current_found;

    //thread and mutex 
    std::thread* read_thread;
    std::mutex* mutex;

    // Function : set the current values of a joint as non found
    void set_join_not_found();

    // Thread function : listen for the joint_states message topic after subcription
    void joint_states_listener(ros::NodeHandle *nh);

    // Callback function : when a joint_states message arrives, save the values
    void joint_states_callback(const sensor_msgs::JointStateConstPtr& msg);


    // Class function : get in the current_* variable the values for the joint joint_name
    //  --> found variable is 1 if found, 0 if not 
    int get_joint_sate(std::string joint_name);
    
    // Callback waiting for requests from clients
    bool callback_joint_states(pr2_joint_states_listener::pr2_joint_states_listener::Request &request,
                               pr2_joint_states_listener::pr2_joint_states_listener::Response &response);

    /// ----------------------- PUBLIC ------------------------- ///

    public:
    
    // Constructor of the class
    LastestJointStates(ros::NodeHandle *nh);

    // Destructor of the class
    ~LastestJointStates();

};



#endif // ifndef PR2_JOINT_STATES_LISTENER_SERVER_CLASS_H