//RN
//17/02/2017

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Matrix.h>

class Playfile_gearbox {
public:
	std::string fname;
	std::vector<double> gripper_angs1, gripper_angs2;
	std::vector<double> arrival_times;
	std::vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
	int data_size;	
	//std::vector<double> new_arrival_times;
	std::vector<double> new_arrival_times;
	bool change_gear(double& scale, std::vector<double> &arrival_times_, int data_size_);
	//bool interpolate (int& line_, double& interval_, std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int data_size_);
	bool interpolate (std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int data_size_);

	
private:


};






