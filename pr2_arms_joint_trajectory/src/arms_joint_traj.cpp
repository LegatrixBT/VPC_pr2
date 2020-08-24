/*
 * File: arms_joint_traj.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Friday, 5th June 2020 12:42:19 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Friday, 14th August 2020 12:02:10 pm
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


//! Initialize the action client and wait for action server to come up
//! Initialize then the joint states client 
Arms_joint_traj::RobotArms::RobotArms(ros::NodeHandle* nh){

    // tell the action client that we want to spin a thread by default
    right_arm_traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
    left_arm_traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while( (!right_arm_traj_client_->waitForServer(ros::Duration(5.0))) && (!left_arm_traj_client_->waitForServer(ros::Duration(5.0))) ){
    ROS_INFO("Waiting for the joint_trajectory_action server for both arms");
    }

    // Initialize the joint_states_client 
    joint_states_client_ = new JointStatesClient(nh);

    //Init the arms goal 
    Arms_joint_traj::RobotArms::init_arm_goal();

}

//! Clean up the action clients and joint states client
Arms_joint_traj::RobotArms::~RobotArms(){
    delete right_arm_traj_client_;
    delete left_arm_traj_client_;
    delete joint_states_client_;
}

//! Sends the command to start a given trajectory for a given arm
void Arms_joint_traj::RobotArms::startArmTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool r_arm, float delay){
    // When to start the trajectory: delay seconds from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
    if (r_arm) 
        right_arm_traj_client_->sendGoal(goal);
    else
        left_arm_traj_client_->sendGoal(goal);
}

void Arms_joint_traj::RobotArms::startBothTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goalR, pr2_controllers_msgs::JointTrajectoryGoal goalL, float delay){
    // When to start the trajectory: delay seconds from now
    goalL.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
    goalR.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
    left_arm_traj_client_->sendGoal(goalL);
    right_arm_traj_client_->sendGoal(goalR);
}

void Arms_joint_traj::RobotArms::startBothTrajectory(const float delay){
    // When to start the trajectory: delay seconds from now
    l_arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
    r_arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
    left_arm_traj_client_->sendGoal(l_arm_goal);
    right_arm_traj_client_->sendGoal(r_arm_goal);
}

//! Get the states of the arms and feed the both arrays with the current arms positions
void Arms_joint_traj::RobotArms::get_arms_joint_position(std::array<double,NB_JOINTS_PR2_UNIQUE_ARM>& r_arm_pos, std::array<double,NB_JOINTS_PR2_UNIQUE_ARM>& l_arm_pos){

    // Get the current states from the robot 
    std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES> joint_states;
    joint_states_client_->call_serivce_last_arms_joint_states(joint_states);

    // 0-->6 r_arm 7--> 13 l_arm 
    std::move(joint_states[JOINT_POSITION].begin(), joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_arm_pos.begin()); 
    std::move(joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_POSITION].end(), l_arm_pos.begin()); 

}


pr2_controllers_msgs::JointTrajectoryGoal Arms_joint_traj::RobotArms::arm_joint_movement_user_verif(bool r_arm){
    
    // goal that will be returned by the function
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    // vectors where will be stored respectively joint names and joint states
    std::vector<std::string> arm_joint_names;
    std::vector<std::vector<double>> arm_joint_states;
    // vector q of the joints positions (rad) 
    std::vector<double> q;

    if(r_arm){
        arm_joint_names = JOINTS_NAME_PR2_R_ARMS;
    } else{
        arm_joint_names = JOINTS_NAME_PR2_L_ARMS;
    }

    // Get all the joints states of the joints named int the arm_joint_names vector
    arm_joint_states = joint_states_client_->call_serivce_last_joint_states(arm_joint_names);
    
    
    for (int i = 0; i < NB_JOINTS_PR2_UNIQUE_ARM; i++){
        // store in the q vector the positions of each joints
        q.push_back(arm_joint_states[JOINT_POSITION][i]);
        // add the joints names as a goal 
        goal.trajectory.joint_names.push_back(arm_joint_names[i]);
    }
    
    // Change the number of waypoints ( start and goal )
    goal.trajectory.points.resize(2);
    int idx = 0;

    // add the current joint position as start for the movement
    // for posiitons and velocities
    goal.trajectory.points[idx].positions.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    goal.trajectory.points[idx].velocities.resize(NB_JOINTS_PR2_UNIQUE_ARM);

    for (int joint = 0; joint < NB_JOINTS_PR2_UNIQUE_ARM ; joint++){
        goal.trajectory.points[idx].positions[joint] = arm_joint_states[JOINT_POSITION][joint];
        goal.trajectory.points[idx].velocities[joint] = arm_joint_states[JOINT_VELOCITY][joint];
    }

    double val;
    int joint; 
    if(r_arm)
        std::cout << "------------------RIGHT-------------\n\n";
    else
        std::cout << "------------------LEFT-------------\n\n";

    std::cout << "q --> "; std::cin >> joint; std::cout << "value --> "; std::cin >> val;

    q[joint] = val;
///////////----------------------------------------////////////////////
    
    Kinematic::GeometricModel model;

    // Evalutation the geometric model 
    model.evaluate(q,r_arm);
    
    Eigen::VectorXd nextPos(4);
    Eigen::VectorXd currPos(4);
    Eigen::RowVector3d currRPY;
    Eigen::RowVector3d nextRPY;

    currPos << 0,0,0,1;
    currRPY << 0,0,0;
    nextPos << 0,0,0,1;
    nextRPY << 0,0,0;
    
    // To evaluate the model, get the position in of different frame relative to another
    nextPos = model.TBEE() * currPos;
    model.getPRY(nextRPY,model.TBEE());
    
    std::cout << "current: " << currPos.transpose() << "\nnextPos : " << nextPos.transpose() << std::endl;
    std::cout << "current: " << currRPY << "\nnextRPY : " << nextRPY << std::endl;

    // For the second point, fill the positions and velocities with the q values 
    idx ++;
    goal.trajectory.points[idx].positions.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    goal.trajectory.points[idx].velocities.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    for (int joint = 0; joint < NB_JOINTS_PR2_UNIQUE_ARM ; joint++){
        goal.trajectory.points[idx].positions[joint] = q[joint];
        goal.trajectory.points[idx].velocities[joint] = arm_joint_states[JOINT_VELOCITY][joint];
    }

    // Then return the goal filled with the new values 

    return goal;

}

//! Get the states of the arms and feed the both arrays with the current arms positions
void Arms_joint_traj::RobotArms::get_arms_joint_position(pr2_controllers_msgs::JointTrajectoryGoal& r_goal, pr2_controllers_msgs::JointTrajectoryGoal& l_goal){

    // Get the current states from the robot 
    std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES> joint_states;
    joint_states_client_->call_serivce_last_arms_joint_states(joint_states);

    // 0-->6 r_arm 7--> 13 l_arm 
    std::move(joint_states[JOINT_POSITION].begin(), joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_goal.trajectory.points[0].positions.begin()); 
    std::move(joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_POSITION].end(), l_goal.trajectory.points[0].positions.begin()); 

}

//! Get the states of the arms and feed the both arrays with the current arms velocities
void Arms_joint_traj::RobotArms::get_arms_joint_velocities(pr2_controllers_msgs::JointTrajectoryGoal& r_goal, pr2_controllers_msgs::JointTrajectoryGoal& l_goal){

    // Get the current states from the robot 
    std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES> joint_states;
    joint_states_client_->call_serivce_last_arms_joint_states(joint_states);

    // 0-->6 r_arm 7--> 13 l_arm 
    std::move(joint_states[JOINT_VELOCITY].begin(), joint_states[JOINT_VELOCITY].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_goal.trajectory.points[0].velocities.begin()); 
    std::move(joint_states[JOINT_VELOCITY].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_VELOCITY].end(), l_goal.trajectory.points[0].velocities.begin()); 

}

//! Get the states of the arms and feed the both arrays with the current arms infos (position and velocities)
void Arms_joint_traj::RobotArms::get_arms_joint_infos(pr2_controllers_msgs::JointTrajectoryGoal& r_goal, pr2_controllers_msgs::JointTrajectoryGoal& l_goal){

    // Get the current states from the robot 
    std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES> joint_states;
    joint_states_client_->call_serivce_last_arms_joint_states(joint_states);

    // 0-->6 r_arm 7--> 13 l_arm 
    std::move(joint_states[JOINT_POSITION].begin(), joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_goal.trajectory.points[0].positions.begin()); 
    std::move(joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_POSITION].end(), l_goal.trajectory.points[0].positions.begin()); 
    
    std::move(joint_states[JOINT_VELOCITY].begin(), joint_states[JOINT_VELOCITY].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_goal.trajectory.points[0].velocities.begin()); 
    std::move(joint_states[JOINT_VELOCITY].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_VELOCITY].end(), l_goal.trajectory.points[0].velocities.begin()); 

}
//! update the arms goal with the current arms infos (position and velocities)
void Arms_joint_traj::RobotArms::update_arms_joint_infos(){

    // Get the current states from the robot 
    std::array<std::array<double,NB_JOINTS_PR2_ARMS>,NB_JOINT_STATES> joint_states;
    joint_states_client_->call_serivce_last_arms_joint_states(joint_states);

    // 0-->6 r_arm 7--> 13 l_arm 
    std::move(joint_states[JOINT_POSITION].begin(), joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_arm_goal.trajectory.points[0].positions.begin()); 
    std::move(joint_states[JOINT_POSITION].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_POSITION].end(), l_arm_goal.trajectory.points[0].positions.begin()); 
    
    std::move(joint_states[JOINT_VELOCITY].begin(), joint_states[JOINT_VELOCITY].begin() + NB_JOINTS_PR2_UNIQUE_ARM, r_arm_goal.trajectory.points[0].velocities.begin()); 
    std::move(joint_states[JOINT_VELOCITY].begin() + NB_JOINTS_PR2_UNIQUE_ARM, joint_states[JOINT_VELOCITY].end(), l_arm_goal.trajectory.points[0].velocities.begin()); 

}

//! Get the state of unique arm and feed the array with the current arm info (position or velocities)
std::vector<double> Arms_joint_traj::RobotArms::get_arm_joint_info(const bool r_arm, const bool position){
    std::vector<double> values(NB_JOINTS_PR2_UNIQUE_ARM);
    // update the infos of the arms 
    Arms_joint_traj::RobotArms::update_arms_joint_infos();
    
    // then return the appropriate vector
    if(r_arm){
        if(position){
            std::copy(r_arm_goal.trajectory.points[0].positions.begin(), r_arm_goal.trajectory.points[0].positions.begin() + NB_JOINTS_PR2_UNIQUE_ARM, values.begin() );
        }else{
            std::copy(r_arm_goal.trajectory.points[0].velocities.begin(), r_arm_goal.trajectory.points[0].velocities.begin() + NB_JOINTS_PR2_UNIQUE_ARM, values.begin() );
        }
    }else{ //////////////////ajouter au .h et tester 
        if(position){
            std::copy(l_arm_goal.trajectory.points[0].positions.begin(), l_arm_goal.trajectory.points[0].positions.begin() + NB_JOINTS_PR2_UNIQUE_ARM, values.begin() );
        }else{
            std::copy(l_arm_goal.trajectory.points[0].velocities.begin(), l_arm_goal.trajectory.points[0].velocities.begin() + NB_JOINTS_PR2_UNIQUE_ARM, values.begin() );
        }
    }
    return values;
}

//! Get the states of the arms (after update) and feed the both arrays with the current arms infos (position and velocities)
const void Arms_joint_traj::RobotArms::get_arms_joints_info(std::vector<std::vector<double>>& l_values, std::vector<std::vector<double>>& r_values){
    
    // update the infos of the arms 
    Arms_joint_traj::RobotArms::update_arms_joint_infos();
    
    // requires to be resize to be to fit the sizes
    l_values.resize(2);
    l_values[JOINT_POSITION].resize(NB_JOINTS_PR2_UNIQUE_ARM);
    l_values[JOINT_VELOCITY].resize(NB_JOINTS_PR2_UNIQUE_ARM);
    r_values.resize(2);
    r_values[JOINT_POSITION].resize(NB_JOINTS_PR2_UNIQUE_ARM);
    r_values[JOINT_VELOCITY].resize(NB_JOINTS_PR2_UNIQUE_ARM);

    // then add the values into the appropriate vector 
    std::copy(r_arm_goal.trajectory.points[0].positions.begin(), r_arm_goal.trajectory.points[0].positions.end(), r_values[JOINT_POSITION].begin() );
    std::copy(l_arm_goal.trajectory.points[0].positions.begin(), l_arm_goal.trajectory.points[0].positions.end(), l_values[JOINT_POSITION].begin() );

    std::copy(r_arm_goal.trajectory.points[0].velocities.begin(), r_arm_goal.trajectory.points[0].velocities.end(), r_values[JOINT_VELOCITY].begin() );
    std::copy(l_arm_goal.trajectory.points[0].velocities.begin(), l_arm_goal.trajectory.points[0].velocities.end(), l_values[JOINT_VELOCITY].begin() );
    
}

//! Get the states of the arms (after update) and feed the both arrays with the current arms infos (position and velocities)
const void Arms_joint_traj::RobotArms::get_arms_joints_info(const bool position, Eigen::VectorXd& l_values, Eigen::VectorXd& r_values){
    
    // update the infos of the arms 
    Arms_joint_traj::RobotArms::update_arms_joint_infos();
    
    // requires to be resize to be to fit the sizes
    l_values.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    r_values.resize(NB_JOINTS_PR2_UNIQUE_ARM);

    if(position){
        r_values = Eigen::Map<Eigen::Vector<double, NB_JOINTS_PR2_UNIQUE_ARM> >(r_arm_goal.trajectory.points[0].positions.data());
        l_values = Eigen::Map<Eigen::Vector<double, NB_JOINTS_PR2_UNIQUE_ARM> >(l_arm_goal.trajectory.points[0].positions.data());
    }else{
        r_values = Eigen::Map<Eigen::Vector<double, NB_JOINTS_PR2_UNIQUE_ARM> >(r_arm_goal.trajectory.points[0].velocities.data());
        l_values = Eigen::Map<Eigen::Vector<double, NB_JOINTS_PR2_UNIQUE_ARM> >(l_arm_goal.trajectory.points[0].velocities.data());
    }
}


void Arms_joint_traj::RobotArms::init_arm_goal(){

    // Get the joints_name to publish on the corrects topics
    std::array<std::string,NB_JOINTS_PR2_ARMS> arm_joint_names = JOINTS_NAME_PR2_ARMS;

    // Pushing the joints names into the goal
    for(int i = 0; i < NB_JOINTS_PR2_UNIQUE_ARM ; i++){
        r_arm_goal.trajectory.joint_names.push_back(arm_joint_names[i]);
        l_arm_goal.trajectory.joint_names.push_back(arm_joint_names[i + NB_JOINTS_PR2_UNIQUE_ARM]);
    }

    // We will have an unique waypoint in this goal trajectory
    r_arm_goal.trajectory.points.resize(1);
    l_arm_goal.trajectory.points.resize(1);

    // Positions anbd Velocities
    r_arm_goal.trajectory.points[0].positions.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    r_arm_goal.trajectory.points[0].velocities.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    l_arm_goal.trajectory.points[0].positions.resize(NB_JOINTS_PR2_UNIQUE_ARM);
    l_arm_goal.trajectory.points[0].velocities.resize(NB_JOINTS_PR2_UNIQUE_ARM);

    // get the positions and velocities --> avant changement uniquement positions !!! si BUG rechanger ici 
    Arms_joint_traj::RobotArms::get_arms_joint_infos(r_arm_goal, l_arm_goal);

}

void Arms_joint_traj::RobotArms::set_arm_goal(std::vector<double>& q, const bool& r_arm){

    if(q.size() > NB_JOINTS_PR2_UNIQUE_ARM){
        ROS_WARN("You gived more joints thant a single the pr2 arm got ...\n the movement will not be performed !");
        return ; 
    } 

    if(r_arm){
        std::copy(q.begin(), q.end(), r_arm_goal.trajectory.points[0].positions.begin());
    }else{
        std::copy(q.begin(), q.end(), l_arm_goal.trajectory.points[0].positions.begin());
    }

}

void Arms_joint_traj::RobotArms::set_arm_goal(const bool velocity, std::vector<double>& command, const bool& r_arm){

    if(command.size() > NB_JOINTS_PR2_UNIQUE_ARM){
        ROS_WARN("You gived more joints thant a single the pr2 arm got ...\n the movement will not be performed !");
        return ; 
    } 

    if(r_arm){
        
        if(velocity){
            std::copy(command.begin(), command.end(), r_arm_goal.trajectory.points[0].velocities.begin());
        }else{
            std::copy(command.begin(), command.end(), r_arm_goal.trajectory.points[0].positions.begin());
        }

    }else{

        if(velocity){
            std::copy(command.begin(), command.end(), l_arm_goal.trajectory.points[0].velocities.begin());
        }else{
            std::copy(command.begin(), command.end(), l_arm_goal.trajectory.points[0].positions.begin());
        }
    }

}


void Arms_joint_traj::RobotArms::verif_torse_ee(Kinematic::GeometricModel& model, const bool r_arm){

    r_arm ? model.evaluate(r_arm_goal.trajectory.points[0].positions,r_arm) : model.evaluate(l_arm_goal.trajectory.points[0].positions,r_arm);  

    Eigen::Vector4d currPos, nextPos;
    Eigen::RowVector3d currRPY, nextRPY;

    currPos << 0,0,0,1;
    currRPY << 0,0,0;
    nextPos << 0,0,0,1;
    nextRPY << 0,0,0;

    // To evaluate the model, get the position in of different frame relative to another
    nextPos = model.TBEE() * currPos;
    model.getPRY(nextRPY,model.TBEE());
    
    std::cout << "r_arm : " << r_arm << "\n";
    std::cout << "current[XYZ]: " << currPos.transpose() << "\nnextPos : " << nextPos.transpose() << std::endl;
    std::cout << "current[RPY]: " << currRPY << "\nnextRPY : " << nextRPY << "\n\n";

}


//! Returns the current state of the action
actionlib::SimpleClientGoalState Arms_joint_traj::RobotArms::getRState(){
    return right_arm_traj_client_->getState();
}

actionlib::SimpleClientGoalState Arms_joint_traj::RobotArms::getLState(){
    return left_arm_traj_client_->getState();
}

// Overloading the << operator to print the lasts arm joints positions
namespace Arms_joint_traj{
    std::ostream& operator<<(std::ostream& stream, const RobotArms& arms){
        
        stream << "Left arm : [";
        for (auto i = 0; i < NB_JOINTS_PR2_UNIQUE_ARM-1 ; i++) stream << arms.l_arm_goal.trajectory.points[0].positions[i] << ',';
        stream << arms.l_arm_goal.trajectory.points[0].positions[NB_JOINTS_PR2_UNIQUE_ARM-1] << "]\n\n";
        // Then right arm
        stream << "Rigt arm : [";
        for (auto i = 0; i < NB_JOINTS_PR2_UNIQUE_ARM-1; i++) stream << arms.r_arm_goal.trajectory.points[0].positions[i] << ',';
        stream << arms.r_arm_goal.trajectory.points[0].positions[NB_JOINTS_PR2_UNIQUE_ARM-1] << "]\n";
        return stream;
    }
}


