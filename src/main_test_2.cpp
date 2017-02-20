//RN
//16/02/2017
//Designed to test the functions of RN's playfile writer

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
#include <general_purpose_reader/playfile_writer_rn.h>
using namespace std;

int main(int argc, char** argv) {
	//Set up a node.
	ros::init(argc, argv, "main_test_2");
	ros::NodeHandle nh;

	std::string name_test;
	
	int data_size = 0;
	name_test = "cameraspace_home.csp";
	
//	void load_data(std::string& fname_, vector<Eigen::Affine3d> &gripper1_affines_, vector<Eigen::Affine3d> &gripper2_affines_, vector<double> &arrival_times_, vector<double> &gripper_angs1_, vector<double> &gripper_angs2_);
	vector<Eigen::Affine3d> gripper1_affines_test; 
	vector<Eigen::Affine3d> gripper2_affines_test; 
	vector<double> arrival_times_test; 
	vector<double> gripper_angs1_test; 
	vector<double> gripper_angs2_test;

	//cout <<  "test begins" << gripper1_affines_test[0]<< "\n";

	Playfile_loader Loader_trail;
	Playfile_writer_rn Writer_trail;
//test 1
	Loader_trail.load_data(
	name_test,
	gripper1_affines_test, 
	gripper2_affines_test, 
	arrival_times_test, 
	gripper_angs1_test, 
	gripper_angs2_test,
	data_size);
	cout << data_size << "\n";//TODO delete checked
	ROS_INFO("Data loading complete");//TODO delete checked
//void Playfile_writer_rn::write_playfile(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_)

	Writer_trail.write_playfile(gripper1_affines_test, 
	gripper2_affines_test, 
	arrival_times_test, 
	gripper_angs1_test, 
	gripper_angs2_test,
	data_size);
	ROS_INFO("Data writing complete");//TODO delete failed
//
	//Loader_trail.load_data(
	//name_test);
	//int i = 0;

	//cout << "testing arrival times \n" << "There are" << data_size <<" records: \n";
	//for (i=0; i<data_size; i++){
	//	cout <<  "record" << i << ":" << arrival_times_test[i]<< "\n";
	//}
	
	
	return 0;
}



