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

class Playfile_gearbox {
public:
	std::vector<double> new_arrival_times;
	bool change_gear(double& scale, std::vector<double> &arrival_times_, int data_size_);

private:


};






