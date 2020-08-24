/*
 * File: main.cpp
 * Project: Stage UFPE Recife, BR, 2019-2020
 * File Created: Tuesday, 23rd June 2020 2:40:00 pm
 * -----
 * University: Universite Paul Sabatier - Toulouse (UPPSITECH - SRI)
 * Author: Benoit TRINDADE (benoit.trindade31@gmail.com)
 * -----
 * Last Modified: Wednesday, 29th July 2020 5:13:18 pm
 * Modified By: Benoit TRINDADE (benoit.trindade31@gmail.com>)
 * -----
 * Copyright a choisir - 2020 Etudiant
 * -----
 * HISTORY:
 * Date       	By	Comments
 * -----------	---	---------------------------------------------------------
 */






#include <pr2_prediction_model/globalModel.h>

#include <ros/ros.h>

int main(int argc, char** argv){

    // Init the ROS node
    ros::init(argc, argv, "Prediction_model");


    ros::NodeHandle nh;
    bool zConst = true;
    GlobalModel gModel(std::move(zConst));
    //gModel.compute_output();
    std::cout << "main zConst : " << zConst << "\n\n";

    std::cout << "Press enter to terminate";
    std::cin.get();
    

}

 