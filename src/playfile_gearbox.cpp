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

#include <general_purpose_reader/playfile_gearbox.h>

using namespace std;

//Simple Version
//bool Playfile_gearbox::change_gear (double& scale_, std::vector<double> &arrival_times_, int data_size_) {
	//cout << "data_size_ is: "<< data_size_ <<"\n";
//	for (int i=0 ; i< data_size_; i++){
//		new_arrival_times.push_back( arrival_times_[i]/scale_);
	//cout << new_arrival_times[i]<< " Scotland the Brave \n";
//	}
//	ROS_INFO("Scaling Complete!");
//	return 1;
//}

//Advanecd Version
bool Playfile_gearbox::change_gear (double& scale_, std::vector<double> &arrival_times_, int data_size_){
	int line_0, line_n;
	ROS_INFO("GEARBOX..");
	cout << "There are "<< data_size_ << " rows inside this playfile, please specify from which line you wish to scale: \n";
	cin >> line_0;
	cout << "Please specify which line you wish to end scaling: \n";
	//cin.clear();
	cin >> line_n;
	cout << "line 0 is: " <<line_0 << "and line_n is: " << line_n << "\n"; //TODO delete
	double base_t = arrival_times_[line_0 -1 ];

	for (int i =0; i < (line_0); i++) {
		//cout << "arrival_times_[i]" << i << ": " <<arrival_times_[i] <<"\n"; //debug use
		//note that the fisrt row never changes to ensure enough time for gripper to get in position
		new_arrival_times.push_back( arrival_times_[i] ); 

		//cout << "new_arrival_times_[i]" << i << ": " << new_arrival_times[i] <<"\n";
	}

	for (int i = (line_0); i <= (line_n-1); i++) {
		if (line_0 > 1){
		//cout << "arrival_times_[i]" << i << ": " <<arrival_times_[i] <<"\n";
		
		new_arrival_times.push_back( (arrival_times_[i] - base_t)/scale_  + base_t);

		//cout << "new_arrival_times_[i]" << i << ": " <<new_arrival_times[i] <<"\n";
		}
		
		else if (line_0 =1 ){
		new_arrival_times.push_back( (arrival_times_[i]-arrival_times_[0])/scale_  + arrival_times_[0]);}
	}

	for (int i = line_n; i < data_size_; i++) {
		new_arrival_times.push_back( arrival_times_[i] - arrival_times_[line_n - 1] + new_arrival_times[line_n - 1]); }

	ROS_INFO("Scaling Complete!");
	return 1;
}



bool Playfile_gearbox::interpolate (vector<Eigen::Affine3d> &gripper1_affines_, vector<Eigen::Affine3d> &gripper2_affines_, vector<double> &arrival_times_, vector<double> &gripper_angs1_, vector<double> &gripper_angs2_, int data_size_){
	ROS_INFO("cord dump marker 01");
	//cout << line_ << "\n";
	int line_n;
	double interval;

	cout << "Please enter after which line you wish to interpolate some time: \n";
	cin >> line_n;
	cout << "Please enter time in seconds: ";
	cin >> interval;	

	for (int i = 0; i < (line_n); i++){	//ln 0, 1, .. ,line_
		ROS_INFO("LOOP1");
		gripper1_affines.push_back(gripper1_affines_[i]);
		gripper2_affines.push_back(gripper2_affines_[i]);
		new_arrival_times.push_back( arrival_times_[i] ); //TODO COREDUMPED HERE
		gripper_angs1.push_back(gripper_angs1_[i]);
		gripper_angs2.push_back(gripper_angs2_[i]);
		
	}
	//ROS_INFO("cord dump marker 02");
	for (int i = (line_n - 1); i < (line_n); i++){ //copy the designated line
		ROS_INFO("LOOP2");
		gripper1_affines.push_back(gripper1_affines_[line_n - 1]);
		gripper2_affines.push_back(gripper2_affines_[line_n - 1]);
		new_arrival_times.push_back( interval + arrival_times_[i]); //stay here for an extra amount of time (interval_)
		gripper_angs1.push_back(gripper_angs1_[line_n - 1]);
		gripper_angs2.push_back(gripper_angs2_[line_n - 1]);
	}
	//ROS_INFO("cord dump marker 03");
	for (int i = (line_n); i < (data_size_ ); i++){
		ROS_INFO("LOOP3");
		gripper1_affines.push_back(gripper1_affines_[i]);
		gripper2_affines.push_back(gripper2_affines_[i]);
		new_arrival_times.push_back( arrival_times_[i]+interval); 
		gripper_angs1.push_back(gripper_angs1_[i]);
		gripper_angs2.push_back(gripper_angs2_[i]);
	
	}
	data_size = data_size_ + 1;

return 1;
}





