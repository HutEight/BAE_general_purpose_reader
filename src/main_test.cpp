//RN
//14/02/2017

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


#include <string>
#include <vector>

#include <general_purpose_reader/playfile_loader.h>

using namespace std;

int main(int argc, char** argv) {
	//Set up a node.
	ros::init(argc, argv, "main_test");
	ros::NodeHandle nh;

	std::string name_test;
	
	
	name_test = "cameraspace_home.csp";
	
//	void load_data(std::string& fname_, vector<Eigen::Affine3d> &gripper1_affines_, vector<Eigen::Affine3d> &gripper2_affines_, vector<double> &arrival_times_, vector<double> &gripper_angs1_, vector<double> &gripper_angs2_);
	vector<Eigen::Affine3d> gripper1_affines_test; 
	vector<Eigen::Affine3d> gripper2_affines_test; 
	vector<double> arrival_times_test; 
	vector<double> gripper_angs1_test; 
	vector<double> gripper_angs2_test;

	//cout <<  "test begins" << gripper1_affines_test[0]<< "\n";

	Playfile_loader Loader_trail;

	Loader_trail.load_data(
	name_test,
	gripper1_affines_test, 
	gripper2_affines_test, 
	arrival_times_test, 
	gripper_angs1_test, 
	gripper_angs2_test);

	cout <<  "test result" << arrival_times_test[0]<< "\n";
	
	
	
	return 0;
}





