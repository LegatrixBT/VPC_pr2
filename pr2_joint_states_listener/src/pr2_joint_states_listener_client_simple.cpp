/*
 * File: pr2_joint_states_listener_client_simple.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Friday, 15th May 2020 10:14:08 am
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Saturday, 16th May 2020 12:09:47 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 * 
 * 16-05-202020	BT	Basic service, nnot deleted because helps to understand client/server of services ROS  
 */








#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

// Include of the custom server for listen the joint_state topic 
#include "pr2_joint_states_listener/pr2_joint_states_listener.h"


class LastestJointStatesClient{


    private:
    ros::ServiceClient client;

    public: 
    LastestJointStatesClient(ros::NodeHandle *nh){
        // Create a service client with nodeHandle.serviceClient<service_type>(service_name)
        this->client = nh->serviceClient<pr2_joint_states_listener::pr2_joint_states_listener>("pr2_joint_states_listener");
    }

    int get_last_joint_states(){
        // Create a service request
        pr2_joint_states_listener::pr2_joint_states_listener service;
        // affect the value of the request
        std::vector<std::string> names;
        names.push_back("r_shoulder_pan_joint");
        service.request.name = names;
        // Call of the service and do the task
        if(this->client.call(service)){

            ROS_INFO("Le service renvoie [%lf] ", service.response.position[0]); 
            ROS_INFO("L'effort applique est [%lf] ", service.response.effort[0]);
            return 0;
        } else {
            ROS_INFO("Failled to call the service pr2_joint_states_listener");
            return -1;
        }


    }

};

int main(int argc, char** argv){

    ros::init(argc, argv, "pr2_joint_states_listener_client");
    ros::NodeHandle nh_ ;

    LastestJointStatesClient client = LastestJointStatesClient(&nh_);  
    if(client.get_last_joint_states() != 0)
        return -1;
    return 0;
}