/*
 * File: pr2_geometric_model.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Monday, 18th May 2020 12:12:49 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Sunday, 7th June 2020 12:02:57 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 * 
 * 31-05-202020	BT	Fix the TH beetween joint 5 and the camera frame for l_arm (really different than r_arm --> TF visu on Rviz)
 */






#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pr2_controllers_msgs/JointControllerState.h>

// include of the joint_states_listener 
#include <pr2_joint_states_listener/pr2_joint_states_listener_client.h>

//include of Eigen 
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;
typedef LastestJointStates::LastestJointStatesClient JointStatesClient;

// already define in the pr2_joint_states_listener_client, but here as a reminder
#define JOINT_POSITION 0
#define JOINT_VELOCITY 1
#define JOINT_EFFORT 2


// Geometric model of the arms of the Willow's garage pr2 robot
class GeometricModel {

    private: 

    // From Willow Garage pr2 specification
    double forearm_offset     = 0.321;
    double shoulder_offset    = 0.1;
    double upper_arm_offset   = 0.4;

    // Distances to be evaluated more carefully in the future (default values here) 
    // rosrun tf tf_echo /r_wrist_roll_link /r_gripper_tool_frame
    // - Translation: [0.180, 0.000, 0.000]
    double link7_end_eff_offset  = 0.180;

    // Distance between torso_lift_link and first joint  can be mesure using
    // $ rosrun tf tf_echo /torso_lift_link /r_shoulder_pan_link 
    // - Translation: [0.000 , 0.188, 0.000]
    double torso_link1_offset    = 0.188;


    public: 

    Eigen::Matrix4d T01, T12, T23, T34, T45, T56, T67;

    // HTransforms between different joints of the robot that are not directly in the kinematic chain
    Eigen::Matrix4d T7EE, T5C, TB0, TBC, TC5;

    // HTransforms between joint0 and jointX 
    Eigen::Matrix4d T02, T03, T04, T05, T06, T07, T0EE; // to verify the model

    Eigen::Matrix4d TCB, TBEE, TCEE, TEEB, TEEC;

    Eigen::Matrix4Xd TH; 



    // Constructor of the class
    GeometricModel(){}

    ~GeometricModel(){}

    void evaluate(std::vector<double> q, bool right_arm){

        // Define the homogeneous matrix of the pr2_robot according to the joint construction in the urdf file
        // and visualisable using Rviz by showing the TF of joints : 
        // {"r_shoulder_pan_link","r_shoulder_lift_link", "r_upper_arm_roll_link", "r_elbow_flex_link", ...
        //   ...  "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"}

        T01 <<   cos(q[0])  , -sin(q[0])  ,  0           ,  0, //shoulder_offset,
                 sin(q[0])  ,  cos(q[0])  ,  0           ,  0,
                  0         ,  0          ,  1           ,  0,
                  0         ,  0          ,  0           ,  1;

        T12 <<   cos(q[1])  ,  0          ,  sin(q[1])   , shoulder_offset,
                  0         ,  1          ,   0          ,  0,
                -sin(q[1])  ,  0          ,  cos(q[1])   ,  0,
                  0         ,  0          ,   0          ,  1;

        T23 <<    1         ,   0         ,   0          ,  0,
                  0         ,  cos(q[2])  , -sin(q[2])   ,  0, 
                  0         ,  sin(q[2])  ,  cos(q[2])   ,  0,
                  0         ,   0         ,   0          ,  1;
        

        T34 <<   cos(q[3])  ,  0          ,  sin(q[3])   ,  upper_arm_offset,
                  0         ,  1          ,   0          ,  0,
                -sin(q[3])  ,  0          ,  cos(q[3])   ,  0,
                  0         ,  0          ,   0          ,  1;

        // pure rotation around x axis + offset 
        T45 <<    1         ,  0          ,   0          ,  0,
                  0         , cos(q[4])   , -sin(q[4])   ,  0, 
                  0         , sin(q[4])   ,  cos(q[4])   ,  0,
                  0         ,  0          ,   0          ,  1;

        T56 <<   cos(q[5])  ,  0          ,  sin(q[5])   , forearm_offset,
                  0         ,  1          ,   0          ,  0,
                -sin(q[5])  ,  0          ,  cos(q[5])   ,  0,
                  0         ,  0          ,   0          ,  1;
        
        T67 <<    1         ,  0          ,   0          ,  0,
                  0         , cos(q[6])   , -sin(q[6])   ,  0, 
                  0         , sin(q[6])   ,  cos(q[6])   ,  0,
                  0         ,  0          ,   0          ,  1;

        // Description of the homogeneous matrix between the last joint(7) and gripper
        T7EE <<   1         ,   0         ,  0           , link7_end_eff_offset,
                  0         ,   1         ,  0           ,  0,
                  0         ,   0         ,  1           ,  0,
                  0         ,   0         ,  0           ,  1; 
    

        if(right_arm){

            // Description of the homogeneous matrix between robot_base and joint(1) 
            // for the right arm
            TB0  <<   1       ,   0       ,  0 ,  0,
                      0       ,   1       ,  0 ,  -torso_link1_offset,
                      0       ,   0       ,  1 ,  0, 
                      0       ,   0       ,  0 ,  1;

            // Description of the homogeneous matrix between joint(5) and the forearm camera
            // According to the tf_transform between link5 and optical frame we got :
            /* 
            $ rosrun tf tf_echo /r_forearm_roll_link /r_forearm_cam_optical_frame
            - Translation: [0.135, 0.000, 0.044]
            - Rotation: in Quaternion [0.000, 0.483, -0.000, 0.876]
                in RPY (radian) [-0.000, 1.008, -0.000]
                in RPY (degree) [-0.000, 57.750, -0.000]

                Rotation around y-axis so Ry = [ cos(q), 0, sin(q);
                                                0    , 1,  0    ;
                                                -sin(q), 0, cos(q);]
                With q = 1.008 (rad)
                cos(1.008) = 0.53355332021339441
                sin(1.008) = 0.84576643022128928
            */

            T5C  <<  0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                     0                    ,   1       ,  0                   ,  0,
                    -0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                     0                    ,   0       ,  0                   ,  1;
        }
        else {
            // for the left arm
            TB0  <<   1       ,   0       ,  0 ,  0,
                      0       ,   1       ,  0 ,  torso_link1_offset,
                      0       ,   0       ,  1 ,  0, 
                      0       ,   0       ,  0 ,  1;

            /*
            $ rosrun tf tf_echo /l_forearm_roll_link /l_forearm_cam_optical_frame
            - Translation: [0.135, 0.000, 0.044]
            - Rotation: in Quaternion   [0.483, 0.000, 0.876, -0.000]
                    in RPY (radian) [-0.000, -1.008, -3.142]
                    in RPY (degree) [-0.000, -57.750, -180.000]


                Rotation around y and z-axis so Ry = [ cos(y) ,     0    , sin(y);
                                                        0     ,     1    ,  0    ;
                                                      -sin(y) ,     0    , cos(y);]
                                                      
                                                Rz = [ cos(z),  -sin(z)  ,  0    ;
                                                       sin(z),   cos(z)  ,  0    ;
                                                        0    ,      0    ,  1    ;]
                                                Ryz= 
                                            ⎡cos(y)⋅cos(z)   -sin(z)⋅cos(y)  sin(y)    ⎤
                                            ⎢                                          ⎥
                                            ⎢       sin(z)         cos(z)        0     ⎥
                                            ⎢                                          ⎥
                                            ⎣-sin(y)⋅cos(z)  sin(y)⋅sin(z)   cos(y)    ⎦ 
                                            
                With y = 1.008 (rad)                       , z = -pi (-3.142)
                cos(1.008) =  0.53355332021339441          , cos(-pi) = -1
                sin(1.008) =  0.84576643022128928          , sin(-pi) =  0 
            */
 
            T5C  << -0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                     0                    ,  -1       ,  0                   ,  0,      //-0.00000000000000001 for the R_21(x)
                     0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                     0                    ,   0       ,  0                   ,  1;
        }    



        // Apply the successives transformations and store the partials results
        
        T02 = T01 * T12;
        T03 = T02 * T23;
        T04 = T03 * T34;
        T05 = T04 * T45;
        T06 = T05 * T56;
        T07 = T06 * T67;


        T0EE = T07 * T7EE;

        //////////////////////////// peut etre mieux de changer en end effector to base  etc ... (inverser ... )
        // HTransform from robot_base to camera frame
        TBC  = TB0*T05*T5C;
        TCB = TBC.inverse();
        // HTransform from robot_base to end effector
        TBEE = TB0*T07*T7EE;
        TEEB = TBEE.inverse();
        // HTranform from  camera to EndEffector
        //TCEE = (T05*T5C) * TBEE.inverse();
        TC5 = T5C.inverse();
        TCEE = TC5*T56*T67*T7EE; 
        TEEC = TCEE.inverse();

        TH.resize(4,8);
        // Concat matrix
        TH << T01, T02;
        //std::cout << "q(x) : \n" << q[0] << "\n" << q[1] << "\n" << q[2] << "\n" << q[3] << "\n" << q[4] << "\n" << q[5] << "\n" << q[6] << "\n";

    }

    // give the values of the angles using the RPY formalism for homogeneous matrix
    void getPRY(Eigen::RowVector3d& RPY, Eigen::Matrix4d& TH ){

        // yaw : alpha = atan2(r21,r11)
        // pitch beta = atan2(-r31,sqrt(r32²+r33²))
        // roll gamma = atan2(r32,r33)

        RPY[0] = atan2( TH(2,1), TH(2,2));
        RPY[1] = atan2(-TH(2,0), (std::pow( TH(2,1)*TH(2,1) + TH(2,2)*TH(2,2) ,0.5  )) ); 
        RPY[2] = atan2( TH(1,0), TH(0,0)); 

        //std::cout << "RPY: \n" << RPY << "\n aa : \n " << "\n debug getRPYfct \n\n";

    }

    Eigen::Matrix4Xd getH(){
        return TH;
    }



};

class RobotArms {

    private:
    // Action client for the joint trajectory action 
    // used to trigger the arm movement action
    TrajClient* right_arm_traj_client_;
    TrajClient* left_arm_traj_client_;

    // Joint_states_client for read position 
    // and velocities of joints
    JointStatesClient* joint_states_client_;


    public:
    //! Initialize the action client and wait for action server to come up
    //! Initialize then the joint states client 
    RobotArms(ros::NodeHandle* nh){

        // tell the action client that we want to spin a thread by default
        right_arm_traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
        left_arm_traj_client_ = new TrajClient("l_arm_controller/joint_trajectory_action", true);

        // wait for action server to come up
        while( (!right_arm_traj_client_->waitForServer(ros::Duration(5.0))) && (!left_arm_traj_client_->waitForServer(ros::Duration(5.0))) ){
        ROS_INFO("Waiting for the joint_trajectory_action server for both arms");
        }
    
        // Initialize the joint_states_client 
        joint_states_client_ = new JointStatesClient(nh);

    }

    //! Clean up the action clients and joint states client
    ~RobotArms(){
        delete right_arm_traj_client_;
        delete left_arm_traj_client_;
        delete joint_states_client_;
    }

    //! Sends the command to start a given trajectory for a given arm
    void startArmTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal, bool r_arm, float delay){
        // When to start the trajectory: delay seconds from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
        if (r_arm) 
            right_arm_traj_client_->sendGoal(goal);
        else
            left_arm_traj_client_->sendGoal(goal);
    }

    void startBothTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goalR, pr2_controllers_msgs::JointTrajectoryGoal goalL, float delay){
        // When to start the trajectory: delay seconds from now
        goalL.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
        goalR.trajectory.header.stamp = ros::Time::now() + ros::Duration(delay);
        left_arm_traj_client_->sendGoal(goalL);
        right_arm_traj_client_->sendGoal(goalR);
    }

    pr2_controllers_msgs::JointTrajectoryGoal arm_joint_movement(bool r_arm){
        
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
        GeometricModel* model;
        model = new GeometricModel(); 

        // Evalutation the geometric model 
        model->evaluate(q,r_arm);
        
        Eigen::VectorXd nextPos(4);
        Eigen::VectorXd currPos(4);
        Eigen::RowVector3d currRPY;
        Eigen::RowVector3d nextRPY;

        currPos << 0,0,0,1;
        currRPY << 0,0,0;
        nextPos << 0,0,0,1;
        nextRPY << 0,0,0;
        
        // To evaluate the model, get the position in of different frame relative to another
        nextPos = model->TBEE * currPos;
        model->getPRY(nextRPY,model->TBEE);
        
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

        // realease the memory allocated on the heap
        delete model;
        // Then return the goal filled with the new values 
        return goal;

    }


    //! Returns the current state of the action
    actionlib::SimpleClientGoalState getRState(){
        return right_arm_traj_client_->getState();
    }

    actionlib::SimpleClientGoalState getLState(){
        return left_arm_traj_client_->getState();
    }
}; // en of RobotArms class

int main(int argc, char** argv){

    // Init the ROS node
    ros::init(argc, argv, "robot_driver");


    ros::NodeHandle nh;
    RobotArms arms(&nh);
    // Get the state
    //   std::cout << "Etat :" << arms.showState() << std::endl;

    // Start the trajectory
    //std::vector<std::string> joint_names = JOINTS_NAME_PR2_ARMS;
    std::cout << "Debut du run" << std::endl;
    //arms.arms_joint_movement(joint_names);
    arms.startBothTrajectory(arms.arm_joint_movement(true), arms.arm_joint_movement(false), 0);

    while( !arms.getLState().isDone() && ros::ok()){
        usleep(50000);
    }
    //   arm.startTrajectory(arm.armExtensionTrajectory(),0);
    //   std::cout << "Etat :" << arm.showState() << std::endl;
    // Wait for trajectory completion
    //   while(!arm.getState().isDone() && ros::ok())
    //   {
    //     usleep(50000);
    //   }
    std::cout << "Fin du run" << std::endl;

    return 0;
}