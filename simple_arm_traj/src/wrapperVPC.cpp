/*
 * File: wrapperVPC.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Saturday, 2nd May 2020 12:29:26 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Monday, 4th May 2020 1:06:18 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include "../include/simple_arm_traj/wrapperVPC.h"

//----------------------------------------------------------------------------//
wrapperVPC::wrapperVPC(ros::NodeHandle nh)
{
  // Allocate memory
  bufferVF = new float[8];
  bufferZ = new float[4];
  command = new float[3];

  // Get the sampling time
  //ts = vpc.getTs();

  // To guarantee an image is availble
  imAvailable = false;
  VS_on = true;

  // Subscribe to the visual feature topic
  //image_sub = nh.subscribe("visual_features", 1, &wrapperVPC::callbackVF, this);

  // Publisher: control of the base
  pub_base = nh.advertise<geometry_msgs::Twist> ("/mobile_base_controller/cmd_vel",1);

//   // Publisher: control of the head
//   //pub_head = nh.advertise<trajectory_msgs::JointTrajectory> ("/head_controller/command",1);

//   // Timer to control the robot
//   // timerControl = nh.createTimer(ros::Duration(ts), &wrapperVPC::callbackControl, this);

//   // Setup the messages to be published
//   msg_head.joint_names.resize(2);
//   msg_head.joint_names[0] = "head_1_joint";
//   msg_head.joint_names[1] = "head_2_joint";
//   msg_head.points.resize(1);
//   msg_head.points[0].positions.resize(2);
//   msg_head.points[0].time_from_start.nsec = ts*1000000;
//   // msg_head.points[0].velocities.resize(2);

//   // Setup robot state
//   vpc.setState(0.0, 0.0, 0.0, 0.0);

//   // Setup the visual features of reference
//   // vpc.setVFref(-0.000132999994094, 0.000229900004342, -0.000134899994009, -0.000284999987343,
//   //               0.000361000013072, 0.000226100004511,  0.000361000013072, -0.000284999987343);
//   // vpc.setVFref(-0.132999994094, 0.229900004342, -0.134899994009, -0.284999987343,
//   //               0.361000013072, 0.226100004511,  0.361000013072, -0.284999987343);

//   // vpc.setVFref(-0.0873999968171, 0.140599995852, -0.0873999968171, -0.171000003815,
//   //               0.224199995399,  0.140599995852,  0.224199995399,  -0.171000003815);

//   vpc.setVFref(-0.125400006771,  0.216600000858, -0.12729999423, -0.269800007343,
//                 0.362899929285, -0.272633522749,  0.362899929285, 0.214699998498);

//   // Setup the cuda configuration
//   vpc.setupModelDataCuda_f_wrapper();

//   // Setup the solver
//   vpc.setupSolver();

//   // Start the control loop_rate
//   controlLoop();
}

bool wrapperVPC::get_VS_on()
{
  return VS_on;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init (argc, argv, "simpleVPC");
  // ros::NodeHandle nh_sensor;
  ros::NodeHandle nh;

  // ros::CallbackQueue my_queue;

  wrapperVPC wrapper(nh);

  // test de la librairie eigen 
  std::cout << "Debut des test de librairies !" << std::endl;
  Eigen::MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << "Test de Eigen : " << std::endl <<  m << std::endl;
  // test de nlopt
  nlopt_opt opt; 
  std::cout << "Test de nlopt : " << opt << std::endl 
  << "Fin des tests de librairies !" << std::endl;


  // ros::spin();

  ros::Rate loop_rate(5);

  while (ros::ok())
  // for(int idx = 0; idx < 6; idx++)
  {
    // loop_rate.reset();
    ros::spinOnce();

    std::cout << "Test Loop" << std::endl;

    //Wait for the end of the loop
    loop_rate.sleep();

    // cout << "After sleep" << endl;

    // ros::spinOnce();
    if(!wrapper.get_VS_on())
    {
      ros::spinOnce();
      ros::shutdown();
    }
  }

}