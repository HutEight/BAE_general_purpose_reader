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
// TODO check and delete duplicates
#include <general_purpose_reader/playfile_writer_rn.h>
//#include <playfile_loader.h>

using namespace std;





void Playfile_writer_rn::write_playfile(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_){

	std::vector <std::vector <double> > data;
	std::vector <double>  record(21,0);
	nposes = data_size_;
	for (int i = 0; i < nposes; i++){
		des_gripper_affine1 = gripper1_affines_[i];
		R = des_gripper_affine1.linear();
		//note that push_back has been used istead of '='
		x_vecs1.push_back(R.col(0) );
		z_vecs1.push_back(R.col(2) );
		tip_origins1.push_back(des_gripper_affine1.translation() );

		des_gripper_affine2 = gripper2_affines_[i];
		R = des_gripper_affine2.linear();
		x_vecs2.push_back(R.col(0) );
		//y_vecs2[i] = R.col(1);
		z_vecs2.push_back(R.col(2) );
		tip_origins2.push_back(des_gripper_affine2.translation() );

	}
	//After this for loop, we get x&z_vecs1&2, tip_origins1&2, which can then give us x&z_vec1&2 to fill in (type) data alongside with times and angs.
	for (int n = 0; n < nposes; n++) {
		for (int i = 0; i < 3; i++) {
			// record has been initiated as a vector with 21 zeros
			record[i] = tip_origins1[n](i);
			record[i + 3] = x_vecs1[n](i);
			record[i + 6] = z_vecs1[n](i);
			record[i + 10] = tip_origins2[n](i);
			record[i + 13] = x_vecs2[n](i);
			record[i + 16] = z_vecs2[n](i);

		}
		record[9] = gripper_angs1_[n];
		record[19] = gripper_angs2_[n];
		record[20] = arrival_times_[n];
		//data[n][9] = gripper_ang1;
		//data[n][19] = gripper_ang2;
		data.push_back(record);
	}

	//std::string oname;
	cout<< "type name: \n";
	cin >> oname;
	const char *cstr = oname.c_str();
	ofstream writefile(cstr);

	//ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < nposes; n++){

		for (int i = 0; i < 20; i++){
			writefile << data[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data[n][20] << "\n";
	}

}



void Playfile_writer_rn::split_playfile(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_){
	Playfile_writer_rn Writer_obj_part1, Writer_obj_part2;

	std::vector<Eigen::Affine3d> gripper1_affines_part1;
	std::vector<Eigen::Affine3d> gripper2_affines_part1;
	std::vector<double> arrival_times_part1;
	std::vector<double> gripper_angs1_part1;
	std::vector<double> gripper_angs2_part1;

	std::vector<Eigen::Affine3d> gripper1_affines_part2;
	std::vector<Eigen::Affine3d> gripper2_affines_part2;
	std::vector<double> arrival_times_part2;
	std::vector<double> gripper_angs1_part2;
	std::vector<double> gripper_angs2_part2;

	int line_num;

	cout << "Please enter the number of the line (>0) where you wish to break this playfile into two: \n";
	cin >> line_num;

	for (int n = 0; n < line_num; n++){
		gripper1_affines_part1.push_back(gripper1_affines_[n]);
		gripper2_affines_part1.push_back(gripper2_affines_[n]);
		arrival_times_part1.push_back(arrival_times_[n]);
		gripper_angs1_part1.push_back(gripper_angs1_[n]);
		gripper_angs2_part1.push_back(gripper_angs2_[n]);
	}

	for (int n = line_num; n < data_size_; n++){
		gripper1_affines_part2.push_back(gripper1_affines_[n]);
		gripper2_affines_part2.push_back(gripper2_affines_[n]);
		arrival_times_part2.push_back(arrival_times_[n] - arrival_times_[line_num - 1]);
		gripper_angs1_part2.push_back(gripper_angs1_[n]);
		gripper_angs2_part2.push_back(gripper_angs2_[n]);
	}

	ROS_INFO("The playfile has been split into two..");
	ROS_INFO("Generating the playfile for the first part..");
	Writer_obj_part1.write_playfile(gripper1_affines_part1,
	gripper2_affines_part1,
	arrival_times_part1,
	gripper_angs1_part1,
	gripper_angs2_part1,
	line_num);
	int num2 = data_size_ - line_num;
	ROS_INFO("Generating the playfile for the second part..");
	Writer_obj_part2.write_playfile(gripper1_affines_part2,
	gripper2_affines_part2,
	arrival_times_part2,
	gripper_angs1_part2,
	gripper_angs2_part2,
	num2);

}
