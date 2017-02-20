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

	//std::string name_test;
	
	//int data_size = 0;
	// name_test = "cameraspace_home.csp";

	vector<std::string> name_list;
	name_list.clear();
	std::string templine = "not_empty";
	std::string objname; //which is also the name of its corresponding file
	//vector<std::string> objlist;
	//objlist.clear();
	
	int n_entries = 0;

	vector<Playfile_loader> Loader_objects;
	Loader_objects.clear();

	while (templine.length() != 0) {
		cout << "Please type in files names: \n";
		getline(cin, templine);
		name_list.push_back(templine);
		stringstream streamname(templine);
		getline(streamname, objname, '.');
		
		//const char *objname = streamname.c_str();
		//Loader_objects objname = new Loader_objects;
		//Loader_objects[n_entries].load_data(templine);
		//oader_objects streamname = new Loader_objects;

		Playfile_loader Loader_trail;
		Loader_trail.load_data(templine);
		n_entries++;
		if (templine.length() != 0){
			cout << "Loading.. entry #"<< n_entries << ": "<< templine << "\n"<< "\n";
			//cout << "Its loader: "<<objname<<" \n";
		}
	}


	return 0;
}





