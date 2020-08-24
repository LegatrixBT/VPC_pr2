/*
 * File: pr2_joint_states_listener_server.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Tuesday, 12th May 2020 6:07:53 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Monday, 27th July 2020 8:40:46 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 * 
 * 14-05-202020	BT	Adding exceptions catch, thread and mutx lock, documentation
 * 
 * 13-05-202020	BT	Service server returning the named joined state of pr2 robot
 */





#include <pr2_joint_states_listener/pr2_joint_states_listener_server.h>
   
// Function : set the current values of a joint as non found
void LastestJointStates::set_join_not_found(){
    this->current_found = 0;
    this->current_effort = 0.;
    this->current_velocity = 0.;
    this->current_position = 0.;
}

// Thread function : listen for the joint_states message topic after subcription
void LastestJointStates::joint_states_listener(ros::NodeHandle *nh){
    // Subscriber that need a sensor_msgs and perform the computation directly in the class in parameter
    // Note if need more variables in in/out, use boost::bind(...) to perform the callback link
    try{
        this->sub =  nh->subscribe<sensor_msgs::JointState>("joint_states", SUB_QUEUE_SIZE, &LastestJointStates::joint_states_callback, this);
        ros::spin();
    }
    catch(ros::Exception e){
        ROS_ERROR("Error occured while subscribing to the topic joint_states \n[%s]", e.what());
    }


}

// Callback function : when a joint_states message arrives, save the values
void LastestJointStates::joint_states_callback(const sensor_msgs::JointStateConstPtr& msg){
    
    // Lock the thread  
    this->mutex->lock();
    // Read the values
    this->names = msg->name;
    this->position = msg->position;
    this->velocity = msg->velocity;
    this->effort = msg->effort;
    // Unclock for next read
    this->mutex->unlock();

}


// Class function : get in the current_* variable the values for the joint joint_name
//  --> found variable is 1 if found, 0 if not 
int LastestJointStates::get_joint_sate(std::string joint_name){
    
    // if there is no message yet
    if(this->names.empty()){
        ROS_INFO("No robot message of joint_states received yet! Maybe you forget to bringup the robot ?\n");
        this->set_join_not_found();
        return 0; 
    }      

    // Return the information of the joint

    // Lock the thread 
    this->mutex->lock();

    // if the joint_name is found in the joint_name vector(if the string exist in the vector) 
    std::vector<std::string>::iterator it = std::find(this->names.begin(), this->names.end(), joint_name);
    if(it != this->names.end()){
        // Get the index of the name
        int index = std::distance(this->names.begin(), it);
        // copy the values in the current variable of the object
        this->current_effort = this->effort[index];
        this->current_velocity = this->velocity[index];
        this->current_position = this->position[index];
        this->current_found = 1;

        // Unlock the thread before return
        this->mutex->unlock();
        
        return 0;

    } else { // If the joint is not found
        ROS_INFO_STREAM("Joint [" << joint_name << "] not found!");
        this->set_join_not_found();
        // Unlock the thread before return
        this->mutex->unlock();

        return 1;
    }

}

// Callback waiting for requests from clients
bool LastestJointStates::callback_joint_states(pr2_joint_states_listener::pr2_joint_states_listener::Request &request,
                            pr2_joint_states_listener::pr2_joint_states_listener::Response &response){

    // Declare the vectors that will be fill by the server respoonse
    std::vector<unsigned int> found; 
    std::vector<double> position;
    std::vector<double> effort;
    std::vector<double> velocity;

    // If the request specify a joint name, provide the informations
    // using range based loop of c++11 
    // Note : auto instead of std::string permit to automatically detect the type according to declaration  
    for (std::string & joint_name : request.name)  
    {
        // Get in the object the current values for a give joint_name
        this->get_joint_sate(joint_name);
        // Add the values to the corresponding vector
        found.push_back(this->current_found);
        effort.push_back(this->current_effort);
        velocity.push_back(this->current_velocity);
        position.push_back(this->current_position);
    }

    // Then transmit the values to the service 
    response.found = found;
    response.effort = effort;
    response.velocity = velocity;
    response.position = position;

    return true;
}

// Constructor of the class
LastestJointStates::LastestJointStates(ros::NodeHandle *nh){

    // Creation of the service server with the callback linked
    try{
        this->srv = nh->advertiseService("pr2_joint_states_listener", &LastestJointStates::callback_joint_states, this);
        ROS_INFO("The service pr2_joint_states_listener is now online!");

        // Create a new instance of the thread and link the thread at the class function with the current object
        this->read_thread = new std::thread(&LastestJointStates::joint_states_listener, this, nh);
        // Create a new instance of the mutex  
        this->mutex = new std::mutex();

        try{
            // Synchronizes the moment this function returns with the completion of all the operations in the thread
            this->read_thread->join();
            // Note : code after the join is never executed due to the spin in the callback ! 
        }
        catch(std::runtime_error& e){
            ROS_ERROR("Error occured while synchronizing thread [%s]", e.what());
        }
    }
    catch(ros::Exception e){
        ROS_ERROR("Error occured while creating the service pr2_joint_states_listener \n[%s]", e.what());
    }
}

// Destructor of the class
LastestJointStates::~LastestJointStates(){}


/// -------------------- For tests or to be launch as a node directly --------------------------- ///

int main(int argc, char** argv){

    // Initialisation of the node
    try{
        ros::init(argc, argv, "pr2_joint_states_listener_server");
    }
    catch(ros::Exception e ){
        ROS_ERROR("Error occured while creating the node pr2_joint_states_listener_server \n%s", e.what());
    }

    ros::NodeHandle nh_ ;

    try{
        // Creation of the service and deploy callback waiting for requests
        LastestJointStates ljs = LastestJointStates(&nh_);
        // Keep the service server alive  
        ros::spin();
    }
    catch(ros::Exception::runtime_error e){
        ROS_ERROR("Error occured during runtime of pr2_joint_states_listener_server\n[%s]", e.what());
        return 1;
    }


    return 0;
}