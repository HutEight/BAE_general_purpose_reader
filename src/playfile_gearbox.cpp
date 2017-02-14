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
#include <general_purpose_reader/playfile_loader.h>
//#include <playfile_loader.h>

using namespace std;

//typedef vector <double> record_t;
//typedef vector <record_t> data_t;


// see: http://www.cplusplus.com/forum/general/17771/
//-----------------------------------------------------------------------------
// Let's overload the stream input operator to read a list of CSV fields (which a CSV record).
// Remember, a record is a list of doubles separated by commas ','.

istream& operator>>(istream& ins, record_t& record) {
	// make sure that the returned record contains only the stuff we read now
	record.clear();

	// read the entire line into a string (a CSV record is terminated by a newline)
	string line;
	getline(ins, line);

	// now we'll use a stringstream to separate the fields out of the line
	stringstream ss(line);
	string field;
	while (getline(ss, field, ',')) {
		// for each field we wish to convert it to a double
		// (since we require that the CSV contains nothing but floating-point values)
		stringstream fs(field);
		double f = 0.0; // (default value is 0.0)
		fs >> f;

		// add the newly-converted field to the end of the record
		record.push_back(f);
	}

	// Now we have read a single line, converted into a list of fields, converted the fields
	// from strings to doubles, and stored the results in the argument record, so
	// we just return the argument stream as required for this kind of input overload function.
	return ins;
}

//-----------------------------------------------------------------------------
// Let's likewise overload the stream input operator to read a list of CSV records.
// This time it is a little easier, just because we only need to worry about reading
// records, and not fields.

istream& operator>>(istream& ins, data_t& data) {
	// make sure that the returned data only contains the CSV data we read here
	data.clear();

	// For every record we can read from the file, append it to our resulting data
	record_t record;
	while (ins >> record) {
		data.push_back(record);
	}

	// Again, return the argument stream as required for this kind of input stream overload.
	return ins;
}

//Constructor
//Playfile_loader::Playfile_loader(){
	
	
	
	
//}



void Playfile_loader::load_data(std::string& fname_, vector<Eigen::Affine3d> &gripper1_affines_, vector<Eigen::Affine3d> &gripper2_affines_, vector<double> &arrival_times_, vector<double> &gripper_angs1_, vector<double> &gripper_angs2_){
	//ros::NodeHandle nh;
	//fname = fname_;

	

//ln167 ln189 davinci_playfile_cameraspace.cpp ----------
	//open the trajectory file:
	ifstream infile(fname_.c_str());
	if (!infile){ // file couldn't be opened
		cerr << "Error: file could not be opened; halting" << endl;
		exit(1);
	}

	// Here is the data we want.
	data_t data; //TODO double-check

	// Here is the file containing the data. Read it into data.
	infile >> data;

	// Complain if something went wrong.
	if (!infile.eof()) {
		cout << "error reading file!\n";
		//return 1;
	}

	infile.close();

	// Otherwise, list some basic information about the file.
	cout << "CSV file contains " << data.size() << " records.\n";
	
unsigned min_record_size = data[0].size();
	unsigned max_record_size = 0;
	for (unsigned n = 0; n < data.size(); n++) {
		if (max_record_size < data[ n ].size())
			max_record_size = data[ n ].size();
		if (min_record_size > data[ n ].size())
			min_record_size = data[ n ].size();
	}
	if (max_record_size > 21) {
		ROS_WARN("bad file");
		cout << "The largest record has " << max_record_size << " fields.\n";
		//return 1;
	}
	if (min_record_size < 21) {
		ROS_WARN("bad file");
		cout << "The smallest record has " << min_record_size << " fields.\n";
		//return 1;
	}
//----------

	nposes = data.size();


//ln226 ln276 davinci_playfile_cameraspace.cpp ----------
for (int n = 0; n < nposes; n++) {
		// pack trajectory points, one at a time:
		for (int i = 0; i < 3; i++) {
			tip_origin1(i) = data[n][i];
			x_vec1(i) = data[n][i + 3];
			z_vec1(i) = data[n][i + 6];
			tip_origin2(i) = data[n][i + 10];
			x_vec2(i) = data[n][i + 13];
			z_vec2(i) = data[n][i + 16];
		}
		gripper_ang1 = data[n][9];
		gripper_ang2 = data[n][19];
		//cout<<"read: gripper_ang2 = "<<gripper_ang2<<endl;
		arrival_time = data[n][20];
	
		x_vecs1.push_back(x_vec1);
		z_vecs1.push_back(z_vec1);
		y_vec1 = z_vec1.cross(x_vec1); //construct consistent y-vec from x and z vecs'
		y_vecs1.push_back(y_vec1);
		tip_origins1.push_back(tip_origin1);
		gripper_angs1_.push_back(gripper_ang1);
	
		x_vecs2.push_back(x_vec2);
		z_vecs2.push_back(z_vec2);
		y_vec2 = z_vec2.cross(x_vec2);
		y_vecs2.push_back(y_vec2);
		tip_origins2.push_back(tip_origin2);
		gripper_angs2_.push_back(gripper_ang2); 
		//cout<<"gripper_angs2: "<<gripper_angs2[n]<<endl;

		arrival_times_.push_back(arrival_time);
	}
	//organize these as affine poses:
	//Eigen::Affine3d des_gripper_affine1,des_gripper_affine2;
	//vector<Eigen::Affine3d> gripper1_affines,gripper2_affines;
	//Eigen::Matrix3d R;
	for (int i=0;i<nposes;i++) {
		R.col(0) = x_vecs1[i];
		R.col(1) = y_vecs1[i];
		R.col(2) = z_vecs1[i];
		des_gripper_affine1.linear() = R;
		des_gripper_affine1.translation() = tip_origins1[i];
		gripper1_affines_.push_back(des_gripper_affine1);

		R.col(0) = x_vecs2[i];
		R.col(1) = y_vecs2[i];
		R.col(2) = z_vecs2[i];
		des_gripper_affine2.linear() = R;
		des_gripper_affine2.translation() = tip_origins2[i];
		gripper2_affines_.push_back(des_gripper_affine2);
	}
//----------

// Collected: gripper1_affines, gripper2_affines, arrival_times Above 



}


