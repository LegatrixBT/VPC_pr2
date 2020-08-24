/*
 * File: pr2_joint_states_listener_client.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Wednesday, 13th May 2020 9:35:16 am
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Tuesday, 28th July 2020 11:33:57 am
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 * 
 * 19-05-202020	BT	Passing the joint states by reference instead of copy with return statement
 * 
 * 15-05-202020	BT	Adding the left and the right arm separetly
 */






#include <pr2_joint_states_listener/pr2_joint_states_listener_client.h>


// Constructor of the class LastestJointStatesClient
LastestJointStates::LastestJointStatesClient::LastestJointStatesClient(ros::NodeHandle *nh){
    // Create a service client with nodeHandle.serviceClient<service_type>(service_name)
    this->client = nh->serviceClient<pr2_joint_states_listener::pr2_joint_states_listener>("pr2_joint_states_listener");
}

// Destructor of the class LastestJointStatesClient
LastestJointStates::LastestJointStatesClient::~LastestJointStatesClient(){}

/// ---------------------------- METHODS OF THE CLASS --------------------------------------- ///


// (Faster) Request (of both the arms joints) to the service server the states of each articulation and store the result in a static array 
void LastestJointStates::LastestJointStatesClient::call_serivce_last_arms_joint_states(std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES>& joint_states){

    // String vector with both arms joint_names
    std::vector<std::string> joint_names = JOINTS_NAME_PR2_ARMS;
    
    // affect the joint_names values into the request
    service.request.name = std::move(joint_names);

    // Call of the service and do the task
    try{
        if(this->client.call(service)){
            
            //Verification that all the joints has been found 
            for(int i = 0; i < NB_JOINTS_PR2_ARMS; i++){
                if(!service.response.found[i])
                    ROS_INFO_STREAM("The joint [" <<  service.request.name[i] << "] was not found in the server response!");
            }

            // Moving the data into the array
            std::move(service.response.position.begin(), service.response.position.begin() + NB_JOINTS_PR2_ARMS, joint_states[JOINT_POSITION].begin());
            std::move(service.response.velocity.begin(), service.response.velocity.begin() + NB_JOINTS_PR2_ARMS, joint_states[JOINT_VELOCITY].begin());
            std::move(service.response.effort.begin(), service.response.effort.begin() + NB_JOINTS_PR2_ARMS, joint_states[JOINT_EFFORT].begin());

        } else {
            ROS_WARN("Failled to call the service pr2_joint_states_listener\n Maybe the service server is not online or the robot not bringup ?");
            // exit(EXIT_FAILURE); Remove the exit to not break time consuming algo with a lack of info, the previous ones will be conserved
        }
    }
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error when calling call_service_last_joint_states\n[%s]", e.what());
        exit(EXIT_FAILURE);
    }
}

/// ------------------------------------------------------------------------------------------------ ///

// (Faster) Request (of both the forearms joints) to the service server the states of each articulation and store the result in a static array 
void LastestJointStates::LastestJointStatesClient::call_serivce_last_forearms_joint_states(std::array<std::array<double,NB_JOINTS_PR2_FOREARMS>,NB_JOINT_STATES>& joint_states){

    // String vector with both arms joint_names
        std::vector<std::string> joint_names = JOINTS_NAME_PR2_FOREARMS;
    // affect the joint_names values into the request
        service.request.name = std::move(joint_names);


    // Call of the service and do the task
    try{
        if(this->client.call(service)){
            
            //Verification that all the joints has been found 
            for(int i = 0; i < NB_JOINTS_PR2_FOREARMS; i++){
                if(!service.response.found[i])
                    ROS_INFO_STREAM("The joint [" <<  service.request.name[i] << "] was not found in the server response!");
            }

            // Moving the data into the array
            std::move(service.response.position.begin(), service.response.position.begin() + NB_JOINTS_PR2_FOREARMS, joint_states[JOINT_POSITION].begin());
            std::move(service.response.velocity.begin(), service.response.velocity.begin() + NB_JOINTS_PR2_FOREARMS, joint_states[JOINT_VELOCITY].begin());
            std::move(service.response.effort.begin(), service.response.effort.begin() + NB_JOINTS_PR2_FOREARMS, joint_states[JOINT_EFFORT].begin());

        } else {
            ROS_WARN("Failled to call the service pr2_joint_states_listener\n Maybe the service server is not online or the robot not bringup ?");
            exit(EXIT_FAILURE);
        }
    }
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error when calling call_service_last_joint_states\n[%s]", e.what());
        exit(EXIT_FAILURE);
    }
}

/// ------------------------------------------------------------------------------------------------ ///

// (Faster) Request (of both the forearms joints) to the service server the position of each articulation and store the result in a static array 
void LastestJointStates::LastestJointStatesClient::call_serivce_last_forearms_joint_position( std::array<double,NB_JOINTS_PR2_FOREARMS>& joint_positions){

    // String vector with both arms joint_names
        std::vector<std::string> joint_names = JOINTS_NAME_PR2_FOREARMS;
    // affect the joint_names values into the request
        service.request.name = std::move(joint_names);


    // Call of the service and do the task
    try{
        if(this->client.call(service)){
            
            //Verification that all the joints has been found 
            for(int i = 0; i < NB_JOINTS_PR2_FOREARMS; i++){
                if(!service.response.found[i])
                    ROS_INFO_STREAM("The joint [" <<  service.request.name[i] << "] was not found in the server response!");
            }

            // Moving the data into the array
            std::move(service.response.position.begin(), service.response.position.begin() + NB_JOINTS_PR2_FOREARMS, joint_positions.begin());

        } else {
            ROS_WARN("Failled to call the service pr2_joint_states_listener\n Maybe the service server is not online or the robot not bringup ?");
            exit(EXIT_FAILURE);
        }
    }
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error when calling call_service_last_joint_states\n[%s]", e.what());
        exit(EXIT_FAILURE);
    }
}

/// ------------------------------------------------------------------------------------------------ ///

// (Faster) Request (of both the forearms joints) to the service server the velocity of each articulation and store the result in a static array 
void LastestJointStates::LastestJointStatesClient::call_serivce_last_forearms_joint_velocity(std::array<double,NB_JOINTS_PR2_FOREARMS>& joint_velocities){

    // String vector with both arms joint_names
        std::vector<std::string> joint_names = JOINTS_NAME_PR2_FOREARMS;
    // affect the joint_names values into the request
        service.request.name = std::move(joint_names);


    // Call of the service and do the task
    try{
        if(this->client.call(service)){
            
            //Verification that all the joints has been found 
            for(int i = 0; i < NB_JOINTS_PR2_FOREARMS; i++){
                if(!service.response.found[i])
                    ROS_INFO_STREAM("The joint [" <<  service.request.name[i] << "] was not found in the server response!");
            }

            // Moving the data into the array
            std::move(service.response.velocity.begin(), service.response.velocity.begin() + NB_JOINTS_PR2_FOREARMS, joint_velocities.begin());
            
        } else {
            ROS_WARN("Failled to call the service pr2_joint_states_listener\n Maybe the service server is not online or the robot not bringup ?");
            exit(EXIT_FAILURE);
        }
    }
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error when calling call_service_last_joint_states\n[%s]", e.what());
        exit(EXIT_FAILURE);
    }
}

/// ------------------------------------------------------------------------------------------------ ///

// (Longer) Generic (joint) request to the service server the states of the joint_names passed as parameter and store the result in a vector
std::vector<std::vector<double>> LastestJointStates::LastestJointStatesClient::call_serivce_last_joint_states(std::vector<std::string> joint_names){
    
    // affect the joint_names values into the request
    service.request.name = joint_names;
    
    // Call of the service and do the task
    try{
        if(this->client.call(service)){
            
            //Verification that all the joints has been found 
            for(int i=0; i<joint_names.size(); i++){
                if(!service.response.found[i])
                    ROS_INFO_STREAM("The joint [" <<  joint_names[i] << "] was not found in the server response!");
            }

            // Create the array that will be return after being filled
            std::vector<std::vector<double>> joint_states(3, std::vector<double>(joint_names.size()) );

            // Moving the data into the array
            std::move(service.response.position.begin(), service.response.position.begin() + joint_names.size(), joint_states[JOINT_POSITION].begin());
            std::move(service.response.velocity.begin(), service.response.velocity.begin() + joint_names.size(), joint_states[JOINT_VELOCITY].begin());
            std::move(service.response.effort.begin(), service.response.effort.begin() + joint_names.size(), joint_states[JOINT_EFFORT].begin());

            // Return the vectors of pos,vel and eff
            return joint_states;

        } else {
            ROS_WARN("Failled to call the service pr2_joint_states_listener\n Maybe the service server is not online or the robot not bringup ?");
            exit(EXIT_FAILURE);
        }
    }
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error when calling call_service_last_joint_states\n[%s]", e.what());
        exit(EXIT_FAILURE);
    }
}

/// -------------------- For tests or to be launch as a node directly --------------------------- ///

int main(int argc, char** argv){

    // Initialisation of the node
    try{
        ros::init(argc, argv, "pr2_joint_states_listener_client");
    }
    catch(ros::Exception e ){
        ROS_ERROR("Error occured while creating the node pr2_joint_states_listener_client \n%s", e.what());
    }

    // Create the node handler 
    ros::NodeHandle nh_ ;

    // String vector with both arms joint_names
    std::vector<std::string> joint_names = JOINTS_NAME_PR2_ARMS;

    try{
        // Creating the client and send a request to the service server
        LastestJointStates::LastestJointStatesClient client(&nh_);  
        
        // Different tests of the service
        // std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES> joint_states;
        // client->call_serivce_last_arms_joint_states(joint_states);

        // std::array<std::array<double,NB_JOINTS_PR2_FOREARMS>,NB_JOINT_STATES> joint_states;
        // client.call_serivce_last_forearms_joint_states(joint_states);
        std::vector<std::vector<double>> joint_states;
        joint_states = client.call_serivce_last_joint_states(joint_names);
        
        
        for(int i=0; i<NB_JOINTS_PR2_ARMS; i++){
            
            std::cout << " positions["<<i<<"] :" << joint_states[JOINT_POSITION][i]  << " velocity["<<i<<"] :" << joint_states[JOINT_VELOCITY][i] 
            << " effort["<<i<<"] : " <<  joint_states[JOINT_EFFORT][i] << std::endl ;
            // std::cout << " positions["<<i<<"] :" << joint_states[JOINT_POSITION][i]  << " velocity["<<i<<"] :" << joint_states[JOINT_VELOCITY][i] 
            // << " effort["<<i<<"] : " <<  joint_states[JOINT_EFFORT][i] << std::endl ;
        }
        
        return 1;

    }    
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error occured during runtime of pr2_joint_states_listener_client\n[%s]", e.what());
        return 1;
    }
    return 0;
}