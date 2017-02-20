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

//bool Playfile_gearbox::change_gear (double& scale_, std::vector<double> &arrival_times_, int data_size_) {
	//cout << "data_size_ is: "<< data_size_ <<"\n";
//	for (int i=0 ; i< data_size_; i++){
//		new_arrival_times.push_back( arrival_times_[i]/scale_);
	//cout << new_arrival_times[i]<< " Scotland the Brave \n";
//	}
//	ROS_INFO("Scaling Complete!");
//	return 1;
//}

bool Playfile_gearbox::change_gear (double& scale_, std::vector<double> &arrival_times_, int data_size_){
	int line_0, line_n;
	ROS_INFO("GEARBOX..");
	cout << "There are "<< data_size_ << " rows inside this playfile, please specify from which line you wish to scale: \n";
	cin >> line_0;
	cout << "Please specify which line you wish to end scaling: \n";
	cin >> line_n;
	
	for (int i =0; i < (line_0 -1); i++) {
		new_arrival_times.push_back( arrival_times_[i] ); }
	for (int i = (line_0-1); i <= (line_n-1); i++) {
		if (line_0 > 1){
		new_arrival_times.push_back( (arrival_times_[i] - arrival_times_[line_0 -1 ])/scale_  + arrival_times_[line_0 -1 ]);}
		if (line_0 =1 ){
		new_arrival_times.push_back( (arrival_times_[i])/scale_  + arrival_times_[line_0 -1 ]);}}

	for (int i = line_n; i < data_size_; i++) {
		new_arrival_times.push_back( arrival_times_[i] - arrival_times_[line_n - 1] + new_arrival_times[line_n - 1]); }

	ROS_INFO("Scaling Complete!");
	return 1;
}







