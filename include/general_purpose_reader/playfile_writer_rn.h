//RN
//16/02/2017

#include <ros/ros.h>
#include <ros/package.h>
//#include <davinci_kinematics/davinci_joint_publisher.h>
//#include <davinci_kinematics/davinci_kinematics.h>
//#include <davinci_traj_streamer/davinci_traj_streamer.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
//#include <davinci_traj_streamer/trajAction.h>
#include <eigen3/Eigen/src/Core/Matrix.h>


typedef std::vector <double> record_t;
typedef std::vector <record_t> data_t;
std::istream& operator << (std::istream & ins, record_t & record);
std::istream& operator << (std::istream & ins, data_t & data);

class Playfile_writer_rn {
public:
	//std::string fname;
	//std::vector<double> gripper_angs1, gripper_angs2;
	//std::vector<double> arrival_times;
	//std::vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
	//int data_size;
	//Playfile_loader(); //constructor
	std::string oname;
	void write_playfile(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_);
	void split_playfile(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_);

private:
//ln217 ln225 davinci_playfile_cameraspace.cpp
	std::vector<Eigen::Vector3d> x_vecs1, y_vecs1, z_vecs1, tip_origins1;
	std::vector<Eigen::Vector3d> x_vecs2, y_vecs2,z_vecs2, tip_origins2;
	Eigen::Vector3d x_vec1, y_vec1,z_vec1, tip_origin1;
	Eigen::Vector3d x_vec2, y_vec2,z_vec2, tip_origin2;
	//vector<double> gripper_angs1, gripper_angs2;
	double gripper_ang1, gripper_ang2;
	//vector<double> arrival_times;
	double arrival_time;
	int nposes; //TODO: int nposes = data.size() in constructor

//ln259 ln261 davinci_playfile_cameraspace.cpp
	Eigen::Affine3d des_gripper_affine1,des_gripper_affine2;
	//vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
	Eigen::Matrix3d R;



};
