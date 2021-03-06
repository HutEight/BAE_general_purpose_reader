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


typedef std::vector <double> record_t;
typedef std::vector <record_t> data_t;
typedef std::vector<Eigen::Vector3d> vec3d;
std::istream& operator >> (std::istream & ins, record_t & record);
std::istream& operator >> (std::istream & ins, data_t & data);

//Defining a new class: Playfile_loader
class Playfile_loader {
public:
	//std::string fname;
	std::vector<double> gripper_angs1, gripper_angs2;
	std::vector<double> arrival_times;
	std::vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
	int data_size;
	std::vector<Eigen::Vector3d> x_vecs1, y_vecs1, z_vecs1, tip_origins1;
	std::vector<Eigen::Vector3d> x_vecs2, y_vecs2,z_vecs2, tip_origins2;
	int nposes;
	Eigen::Affine3d des_gripper_affine1,des_gripper_affine2;
	Eigen::Matrix3d R;

	void load_data(std::string& fname_);
	void update_origins(vec3d tip_origins1_, vec3d tip_origins2_);

private:
	Eigen::Vector3d x_vec1, y_vec1,z_vec1, tip_origin1;
	Eigen::Vector3d x_vec2, y_vec2,z_vec2, tip_origin2;
	double arrival_time;
	double gripper_ang1, gripper_ang2;

};
