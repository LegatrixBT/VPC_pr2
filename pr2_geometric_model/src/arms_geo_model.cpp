/*
 * File: arms_geo_model.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Friday, 5th June 2020 12:12:31 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Sunday, 9th August 2020 3:44:54 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






// #include "../include/pr2_geometric_model/arms_geo_model.h"
#include <pr2_geometric_model/arms_geo_model.h>

Kinematic::GeometricModel::GeometricModel(){}

Kinematic::GeometricModel::~GeometricModel(){}

void Kinematic::GeometricModel::evaluate(std::vector<double>& q, const bool right_arm){

    // Define the homogeneous matrix of the pr2_robot according to the joint construction in the urdf file
    // and visualisable using Rviz by showing the TF of joints : 
    // {"r_shoulder_pan_link","r_shoulder_lift_link", "r_upper_arm_roll_link", "r_elbow_flex_link", ...
    //   ...  "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"}

    T01_ <<    cos(q[0])  , -sin(q[0])  ,  0           ,  0,
               sin(q[0])  ,  cos(q[0])  ,  0           ,  0,
                0         ,  0          ,  1           ,  0,
                0         ,  0          ,  0           ,  1;

    T12_ <<    cos(q[1])  ,  0          ,  sin(q[1])   , shoulder_offset,
                0         ,  1          ,   0          ,  0,
              -sin(q[1])  ,  0          ,  cos(q[1])   ,  0,
                0         ,  0          ,   0          ,  1;

    T23_ <<      1         ,   0         ,   0          ,  0,
                0         ,  cos(q[2])  , -sin(q[2])   ,  0, 
                0         ,  sin(q[2])  ,  cos(q[2])   ,  0,
                0         ,   0         ,   0          ,  1;
    

    T34_ <<     cos(q[3])  ,  0          ,  sin(q[3])   ,  upper_arm_offset,
                0         ,  1          ,   0          ,  0,
              -sin(q[3])  ,  0          ,  cos(q[3])   ,  0,
                0         ,  0          ,   0          ,  1;

    // pure rotation around x axis + offset 
    T45_ <<      1         ,  0          ,   0          ,  0,
                0         , cos(q[4])   , -sin(q[4])   ,  0, 
                0         , sin(q[4])   ,  cos(q[4])   ,  0,
                0         ,  0          ,   0          ,  1;

    T56_ <<   cos(q[5])    ,  0          ,  sin(q[5])   , forearm_offset,
                0         ,  1          ,   0          ,  0,
            -sin(q[5])    ,  0          ,  cos(q[5])   ,  0,
                0         ,  0          ,   0          ,  1;
    
    T67_ <<      1         ,  0          ,   0          ,  0,
                0         , cos(q[6])   , -sin(q[6])   ,  0, 
                0         , sin(q[6])   ,  cos(q[6])   ,  0,
                0         ,  0          ,   0          ,  1;

    // Description of the homogeneous matrix between the last joint(7) and gripper
    T7EE_ <<     1         ,   0         ,  0           , link7_end_eff_offset,
                0         ,   1         ,  0           ,  0,
                0         ,   0         ,  1           ,  0,
                0         ,   0         ,  0           ,  1; 


    if(right_arm){

        // Description of the homogeneous matrix between robot_base and joint(1) 
        // for the right arm
        TB0_  <<     1       ,   0       ,  0 ,  0,
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

        T5C_  <<     0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                    0                    ,   1       ,  0                   ,  0,
                   -0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                    0                    ,   0       ,  0                   ,  1;
    }
    else {
        // for the left arm
        TB0_  <<     1       ,   0       ,  0 ,  0,
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

        T5C_  <<    -0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                    0                    ,  -1       ,  0                   ,  0,      //-0.00000000000000001 for the R_21(x)
                    0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                    0                    ,   0       ,  0                   ,  1;
    }    



    // Apply the successives transformations and store the partials results
    
    T02_ = T01_ * T12_;
    T03_ = T02_ * T23_;
    T04_ = T03_ * T34_;
    T05_ = T04_ * T45_;
    T06_ = T05_ * T56_;
    T07_ = T06_ * T67_;


    T0EE_ = T07_ * T7EE_;

    //////////////////////////// peut etre mieux de changer en end effector to base  etc ... (inverser ... )
    // HTransform from robot_base to camera frame
    TBC_ = TB0_*T05_*T5C_;
    TCB_ = TBC_.inverse();
    // HTransform from robot_base to end effector
    TBEE_ = TB0_*T07_*T7EE_;
    TEEB_ = TBEE_.inverse();
    // HTranform from  camera to EndEffector
    //TCEE = (T05*T5C) * TBEE.inverse();
    TC5_ = T5C_.inverse();
    TCEE_ = TC5_*T56_*T67_*T7EE_; 
    TEEC_ = TCEE_.inverse();

    TH_.resize(4,36);
    // Concat matrix
    TH_ << TB0_, T01_, T02_, T03_, T04_, T05_, T06_, T07_, T0EE_;
    //std::cout << "q(x) : \n" << q[0] << "\n" << q[1] << "\n" << q[2] << "\n" << q[3] << "\n" << q[4] << "\n" << q[5] << "\n" << q[6] << "\n";

}

void Kinematic::GeometricModel::evaluate(Eigen::VectorXd& q, const bool right_arm){

    // Define the homogeneous matrix of the pr2_robot according to the joint construction in the urdf file
    // and visualisable using Rviz by showing the TF of joints : 
    // {"r_shoulder_pan_link","r_shoulder_lift_link", "r_upper_arm_roll_link", "r_elbow_flex_link", ...
    //   ...  "r_forearm_roll_link", "r_wrist_flex_link", "r_wrist_roll_link"}

    T01_ <<    cos(q[0])  , -sin(q[0])  ,  0           ,  0,
               sin(q[0])  ,  cos(q[0])  ,  0           ,  0,
                0         ,  0          ,  1           ,  0,
                0         ,  0          ,  0           ,  1;

    T12_ <<    cos(q[1])  ,  0          ,  sin(q[1])   , shoulder_offset,
                0         ,  1          ,   0          ,  0,
              -sin(q[1])  ,  0          ,  cos(q[1])   ,  0,
                0         ,  0          ,   0          ,  1;

    T23_ <<      1         ,   0         ,   0          ,  0,
                0         ,  cos(q[2])  , -sin(q[2])   ,  0, 
                0         ,  sin(q[2])  ,  cos(q[2])   ,  0,
                0         ,   0         ,   0          ,  1;
    

    T34_ <<     cos(q[3])  ,  0          ,  sin(q[3])   ,  upper_arm_offset,
                0         ,  1          ,   0          ,  0,
              -sin(q[3])  ,  0          ,  cos(q[3])   ,  0,
                0         ,  0          ,   0          ,  1;

    // pure rotation around x axis + offset 
    T45_ <<      1         ,  0          ,   0          ,  0,
                0         , cos(q[4])   , -sin(q[4])   ,  0, 
                0         , sin(q[4])   ,  cos(q[4])   ,  0,
                0         ,  0          ,   0          ,  1;

    T56_ <<   cos(q[5])    ,  0          ,  sin(q[5])   , forearm_offset,
                0         ,  1          ,   0          ,  0,
            -sin(q[5])    ,  0          ,  cos(q[5])   ,  0,
                0         ,  0          ,   0          ,  1;
    
    T67_ <<      1         ,  0          ,   0          ,  0,
                0         , cos(q[6])   , -sin(q[6])   ,  0, 
                0         , sin(q[6])   ,  cos(q[6])   ,  0,
                0         ,  0          ,   0          ,  1;

    // Description of the homogeneous matrix between the last joint(7) and gripper
    T7EE_ <<     1         ,   0         ,  0           , link7_end_eff_offset,
                0         ,   1         ,  0           ,  0,
                0         ,   0         ,  1           ,  0,
                0         ,   0         ,  0           ,  1; 


    if(right_arm){

        // Description of the homogeneous matrix between robot_base and joint(1) 
        // for the right arm
        TB0_  <<     1       ,   0       ,  0 ,  0,
                    0       ,   1       ,  0 ,  -torso_link1_offset,
                    0       ,   0       ,  1 ,  0, 
                    0       ,   0       ,  0 ,  1;

        T5C_  <<     0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                    0                    ,   1       ,  0                   ,  0,
                   -0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                    0                    ,   0       ,  0                   ,  1;
    }
    else {
        // for the left arm
        TB0_  <<     1       ,   0       ,  0 ,  0,
                    0       ,   1       ,  0 ,  torso_link1_offset,
                    0       ,   0       ,  1 ,  0, 
                    0       ,   0       ,  0 ,  1;

        T5C_  <<    -0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                    0                    ,  -1       ,  0                   ,  0,      //-0.00000000000000001 for the R_21(x)
                    0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                    0                    ,   0       ,  0                   ,  1;
    }    

    // Apply the successives transformations and store the partials results
    
    T02_ = T01_ * T12_;
    T03_ = T02_ * T23_;
    T04_ = T03_ * T34_;
    T05_ = T04_ * T45_;
    T06_ = T05_ * T56_;
    T07_ = T06_ * T67_;

    T0EE_ = T07_ * T7EE_;

    //////////////////////////// peut etre mieux de changer en end effector to base  etc ... (inverser ... )
    // HTransform from robot_base to camera frame
    TBC_ = TB0_*T05_*T5C_;
    TCB_ = TBC_.inverse();
    // HTransform from robot_base to end effector
    TBEE_ = TB0_*T07_*T7EE_;
    TEEB_ = TBEE_.inverse();
    // HTranform from  camera to EndEffector
    //TCEE = (T05*T5C) * TBEE.inverse();
    TC5_ = T5C_.inverse();
    TCEE_ = TC5_*T56_*T67_*T7EE_; 
    TEEC_ = TCEE_.inverse();

    TH_.resize(4,36);
    // Concat matrix
    TH_ << TB0_, T01_, T02_, T03_, T04_, T05_, T06_, T07_, T0EE_;
    //std::cout << "q(x) : \n" << q[0] << "\n" << q[1] << "\n" << q[2] << "\n" << q[3] << "\n" << q[4] << "\n" << q[5] << "\n" << q[6] << "\n";

}

void Kinematic::GeometricModel::evaluate5Joints(Eigen::VectorXd& q, const bool right_arm){

    T01_ <<    cos(q[0])  , -sin(q[0])  ,  0           ,  0,
               sin(q[0])  ,  cos(q[0])  ,  0           ,  0,
                0         ,  0          ,  1           ,  0,
                0         ,  0          ,  0           ,  1;

    T12_ <<    cos(q[1])  ,  0          ,  sin(q[1])   , shoulder_offset,
                0         ,  1          ,   0          ,  0,
              -sin(q[1])  ,  0          ,  cos(q[1])   ,  0,
                0         ,  0          ,   0          ,  1;

    T23_ <<     1         ,   0         ,   0          ,  0,
                0         ,  cos(q[2])  , -sin(q[2])   ,  0, 
                0         ,  sin(q[2])  ,  cos(q[2])   ,  0,
                0         ,   0         ,   0          ,  1;
    

    T34_ <<    cos(q[3])  ,  0          ,  sin(q[3])   ,  upper_arm_offset,
                0         ,  1          ,   0          ,  0,
              -sin(q[3])  ,  0          ,  cos(q[3])   ,  0,
                0         ,  0          ,   0          ,  1;

    // pure rotation around x axis + offset 
    T45_ <<     1         ,  0          ,   0          ,  0,
                0         , cos(q[4])   , -sin(q[4])   ,  0, 
                0         , sin(q[4])   ,  cos(q[4])   ,  0,
                0         ,  0          ,   0          ,  1;

    if(right_arm){

        // Description of the homogeneous matrix between robot_base and joint(1) 
        // for the right arm
        TB0_  <<    1       ,   0       ,  0 ,  0,
                    0       ,   1       ,  0 ,  -torso_link1_offset,
                    0       ,   0       ,  1 ,  0, 
                    0       ,   0       ,  0 ,  1;

        T5C_  <<    0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                    0                    ,   1       ,  0                   ,  0,
                   -0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                    0                    ,   0       ,  0                   ,  1;
    }
    else {
        // for the left arm
        TB0_  <<    1       ,   0       ,  0 ,  0,
                    0       ,   1       ,  0 ,  torso_link1_offset,
                    0       ,   0       ,  1 ,  0, 
                    0       ,   0       ,  0 ,  1;

        T5C_  <<   -0.53355332021339441  ,   0       ,  0.84576643022128928 ,  0.135,
                    0                    ,  -1       ,  0                   ,  0,      //-0.00000000000000001 for the R_21(x)
                    0.84576643022128928  ,   0       ,  0.5335532021339441  ,  0.044,
                    0                    ,   0       ,  0                   ,  1;
    }    

    T02_ = T01_ * T12_;
    T03_ = T02_ * T23_;
    T04_ = T03_ * T34_;
    T05_ = T04_ * T45_;

    // HTransform from robot_base to camera frame
    TBC_ = TB0_*T05_*T5C_;
    TCB_ = TBC_.inverse();
    TC5_ = T5C_.inverse();

    TH_.resize(4,24);
    // Concat matrix
    TH_ << TB0_, T01_, T02_, T03_, T04_, T05_;
}

void Kinematic::GeometricModel::getPRY(Eigen::RowVector3d& RPY, const Eigen::Matrix4d TH ) const{
    
    // yaw : alpha = atan2(r21,r11)
    // pitch beta = atan2(-r31,sqrt(r32²+r33²))
    // roll gamma = atan2(r32,r33)

    RPY[0] = atan2( TH(2,1), TH(2,2));
    RPY[1] = atan2(-TH(2,0), (std::pow( TH(2,1)*TH(2,1) + TH(2,2)*TH(2,2) ,0.5  )) ); 
    RPY[2] = atan2( TH(1,0), TH(0,0)); 

    //std::cout << "RPY: \n" << RPY << "\n aa : \n " << "\n debug getRPYfct \n\n";

}

Eigen::Matrix4Xd Kinematic::GeometricModel::getH() const {
    return TH_;
}
