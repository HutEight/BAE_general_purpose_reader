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

#include <general_purpose_reader/playfile_loader.h>
#include <general_purpose_reader/playfile_writer_rn.h>
#include <general_purpose_reader/playfile_gearbox.h>

using namespace std;


bool setFlag() {
cout<<"Do you wish to Proceed and Select Mode (press 1) or Restart (press 2) or Quit (press 0)? \n";
int wish;
cin >> wish;
switch (wish)
{
	case 1:
		return 1;
	break;
	case 2:
		return 0;
	break;
	case 0:
		exit(0);
	break;


}


}

int main(int argc, char** argv) {
	ros::init(argc, argv, "general_purpose_reader");
	ros::NodeHandle nh;
//int en = 1;
while (ros::ok()){
//First enter and store a list of playfile names
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	ROS_INFO("GENERAL PURPOSE PLAYFILE READER");
	cout << "***Please type in the playfile names and hit Enter to finish. ***\n"<<"\n";
	std::string templine = "not_empty";

	int n_entries = 0;
	vector<std::string> name_list;
	name_list.clear();


	while (templine.length() != 0) {
		cout << "Please enter a playfile name: \n";
		getline(cin, templine);
		name_list.push_back(templine); //name_list
		stringstream streamname(templine);
		n_entries++;
		if (templine.length() != 0){
			cout << "Loading.. entry #"<< n_entries << ": "<< templine << "\n"<< "\n";
		}
	}

	cout << "You have entered "<< --n_entries << " files. \n";

//Then choose 1. Gearbox, 2. Playfile Merger


	int mode_code;

	while (setFlag()){
	ROS_INFO("Filelist:");

	Playfile_loader Loader_obj;	//will not be put into switch case
	Playfile_writer_rn Writer_obj;
	Playfile_gearbox Gearbox_obj;

	for (int i=0; i < n_entries; i++){
		cout << i << ". "<< name_list[i]<< "\n";
		}
	cout << "\n";
	cout << "Please Choose Mode: \n" << "Press 1 for Gearbox;\n" << "Press 2 for Playfile Merger; \n"<< "Press 3 for time interpolation. \n"<< "Press 9 to change mode / return. \n"<<"\n";
	cin >> mode_code;
	switch (mode_code)
	{
		case 1://Gearbox
		ROS_INFO("Choose one file to scale:");
		n_entries++;
		int choice;
		cin >> choice;
		Loader_obj.load_data(name_list[choice]);
		cout << "Please enter scale: x";
		double scale;

		cin >> scale;
		Gearbox_obj.change_gear(scale, Loader_obj.arrival_times, Loader_obj.data_size);

		Writer_obj.write_playfile(Loader_obj.gripper1_affines, Loader_obj.gripper2_affines, Gearbox_obj.new_arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);

		name_list.push_back(Writer_obj.oname);
		//cout << Writer_obj.oname << "\n";
		ROS_INFO("New playfile saved!");
		//ROS_INFO("MARKER 2");
		break;

		case 2://Merger
			ROS_WARN("This Feature has not been added yet.");




		break;

		case 3://Interpolate extra time between lines
		ROS_INFO("Ready to interpolate time.");
		ROS_INFO("Choose one file to interpolate time:");
		double length;
		int line_num;
		n_entries++;
		int choice2;
		cin >> choice2;
		Loader_obj.load_data(name_list[choice2]);

		//bool interpolate (int& line_, double& interval_, std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int data_size_);
		Gearbox_obj.interpolate(Loader_obj.gripper1_affines, Loader_obj.gripper2_affines, Loader_obj.arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);
		Writer_obj.write_playfile(Gearbox_obj.gripper1_affines,Gearbox_obj.gripper2_affines, Gearbox_obj.new_arrival_times, Gearbox_obj.gripper_angs1, Gearbox_obj.gripper_angs2,Gearbox_obj.data_size);

		name_list.push_back(Writer_obj.oname);
		//cout << Writer_obj.oname << "\n";
		ROS_INFO("New playfile saved!");
		//ROS_INFO("MARKER 2");
		break;



		case 9://Null, RETURN
			ROS_WARN("Returning..");
		break;

	}
	}







}
}
