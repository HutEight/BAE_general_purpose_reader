//RN
//HMS QUEEN ELIZABETH
//27/03/2017

#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <eigen3/Eigen/src/Core/Matrix.h>

using namespace std;


int main(int argc, char** argv) {
  ros::init(argc, argv, "draw_circle_jointspace");
	ros::NodeHandle nh;

  double joint_1_0, joint_2_0, joint_3_0, t;
  double joint_1, joint_2;
  double init_radius = 0.5;
  double arrival_time;

  t = 30;
  std::vector <std::vector <double> > data;
  std::vector <double>  record(15,0);
  //nposes = data_size_;

  cout << "You will be creating a circle-drawing jointspace playfile \n";

  cout << "The playfile will be consisted of 72 points froming the circle. \n";
  double PI = 3.14159;
  for (int n = 0; n < 72; n++){
    joint_1 = init_radius*cos(n*2*PI/72);
    joint_2 = init_radius*sin(n*2*PI/72);

    arrival_time = 15 + n * t/72;
    record[0] = joint_1;
    record[1] = joint_2;
    record[14] = arrival_time;

    data.push_back(record);
  }
  std::string oname;
  cout<< "type name: \n";
  cin >> oname;
  const char *cstr = oname.c_str();
  ofstream writefile(cstr);

  ROS_INFO("The circle generation is complete, sending data to embeded Playfile Writer. ");
  //data_size = 72;

  //ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < 72; n++){

		for (int i = 0; i < 15; i++){
			writefile << data[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data[n][15] << "\n";
	}

}
