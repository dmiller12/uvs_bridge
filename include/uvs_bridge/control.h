
#ifndef CONTROL_H
#define CONTROL_H


#include <ros/ros.h>
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <Eigen/Dense>

class UVSControl {
    public:
        ArmControl *arm;
        bool rest;
        int dof;
        int total_joints;
        int move_step(void);

}


#endif