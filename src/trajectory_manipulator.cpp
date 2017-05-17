//RN
//23/02/2017

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

#include <general_purpose_reader/trajectory_manipulator.h>

#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>

#include <general_purpose_reader/playfile_writer_rn.h>


using namespace std;

//Caution: You have to first instanciate a playfile_loader object and order it to load a playfile before you can use this manipulator to move the gripper tips.
//Caution: The new_tip_origins will be stored in the trajectory_manipulator object and thus in the main function you will have to pass them to the playfile_loader_obj.update_origins()

void Trajectory_manipulator::move_gripper_tip(vec3d tip_origins1_, vec3d tip_origins2_, int nposes_){

  double distance, drt_x, drt_y, drt_z;
  int gripper_choice;
  Eigen::Vector3d direction(3);
  cout <<"You will be moving the gripper_tips \n"<<"Which gripper would you like to move? \n";
  cout <<"Press 1 for Right Grippe; \nPress 2 for Left Gripper; \nPress 3 for both Grippers:\n";
  cin >> gripper_choice;
  cout <<"Now please indicate the drection by typing a vector (x, y, z) in the Portal Space. \n";
  cout <<"x: ";
  cin >> drt_x;
  cout <<"y: ";
  cin >> drt_y;
  cout <<"z: ";
  cin >> drt_z;
  cout << "And please type in the distance you wish the gripper to move (in centimeters): \n";
  cin >> distance;
  direction << drt_x, drt_y, drt_z;
  offset_x = 0.01*distance*drt_x/direction.norm();
  offset_y = 0.01*distance*drt_y/direction.norm();
  offset_z = 0.01*distance*drt_z/direction.norm();

  switch (gripper_choice)
  {
    case 1://Right Gripper, first 10 columns of trajectory files
      new_tip_origins1 = tip_origins1_;
      new_tip_origins2 = tip_origins2_;
      for (int n=0; n < nposes_; n++){
        //for (int i=0, i < 3; i++){

          cout << "offset_z is "<< offset_z <<"\n";


          new_tip_origins1[n][0] = new_tip_origins1[n][0] + offset_x;
          new_tip_origins1[n][1] = new_tip_origins1[n][1] + offset_y;
          new_tip_origins1[n][2] = new_tip_origins1[n][2] + offset_z;

        //}
      }
    break;
    case 2://Left Gripper, Column11 - 20 of trajectory files
      new_tip_origins1 = tip_origins1_;
      new_tip_origins2 = tip_origins2_;
      for (int n=0; n < nposes_; n++){
        //for (int i=0, i < 3; i++){
          new_tip_origins2[n][0] = new_tip_origins2[n][0] + offset_x;
          new_tip_origins2[n][1] = new_tip_origins2[n][1] + offset_y;
          new_tip_origins2[n][2] = new_tip_origins2[n][2] + offset_z;
        //}
      }
    break;
    case 3://Both Grippers
      new_tip_origins1 = tip_origins1_;
      new_tip_origins2 = tip_origins2_;
      for (int n=0; n < nposes_; n++){
        //for (int i=0, i < 3; i++){
          new_tip_origins1[n][0] = new_tip_origins1[n][0] + offset_x;
          new_tip_origins1[n][1] = new_tip_origins1[n][1] + offset_y;
          new_tip_origins1[n][2] = new_tip_origins1[n][2] + offset_z;
          new_tip_origins2[n][0] = new_tip_origins2[n][0] + offset_x;
          new_tip_origins2[n][1] = new_tip_origins2[n][1] + offset_y;
          new_tip_origins2[n][2] = new_tip_origins2[n][2] + offset_z;
        //}
      }
    break;

  }

}

void Trajectory_manipulator::draw_circle(){

  double x_0, y_0, z_0, r, t;
  double x, y;
  cout << "You will be creating a circle-drawing playfile \n";
  cout << "Please specify the centre in the 3D space (x, y, z): \n";
  cout << "x: "; cin >> x_0;
  cout << "y: "; cin >> y_0;
  cout << "z: "; cin >> z_0;
  cout << "Please specify the radius r (in centimeters): \n";
  cin >> r;
  r = r * 0.01;
  cout << "Please specify a time for the gripper to complete the circle, t: \n";
  cin >> t;
  cout << "The playfile will be consisted of 72 points froming the circle. \n";
  for (int n = 0; n < 72; n++){
    x = x_0 + r*cos(n*2*PI/72);
    y = y_0 + r*sin(n*2*PI/72);

    tip_origin1(0) = x;
    tip_origin1(1) = y;
    tip_origin1(2) = z_0;
    x_vec1(0) = 0;
    x_vec1(1) = 1;
    x_vec1(2) = 0;
    z_vec1(0) = 0;
    z_vec1(1) = 0;
    z_vec1(2) = 1; //Gripper always is pointing down

    tip_origin2(0) = x;
    tip_origin2(1) = y;
    tip_origin2(2) = z_0;
    x_vec2(0) = 0;
    x_vec2(1) = 1;
    x_vec2(2) = 0;
    z_vec2(0) = 0;
    z_vec2(1) = 0;
    z_vec2(2) = 1; //Gripper always is pointing down

    gripper_ang1 = 0;
    gripper_ang2 = 0;
    arrival_time = 15 + n * t/72;

    x_vecs1.push_back(x_vec1);
		z_vecs1.push_back(z_vec1);
		y_vec1 = z_vec1.cross(x_vec1); //construct consistent y-vec from x and z vecs'
		y_vecs1.push_back(y_vec1);
		tip_origins1.push_back(tip_origin1);
		gripper_angs1.push_back(gripper_ang1);

		x_vecs2.push_back(x_vec2);
		z_vecs2.push_back(z_vec2);
		y_vec2 = z_vec2.cross(x_vec2);
		y_vecs2.push_back(y_vec2);
		tip_origins2.push_back(tip_origin2);
		gripper_angs2.push_back(gripper_ang2);

		arrival_times.push_back(arrival_time);
  }
  for (int i=0;i<72;i++) {
    R.col(0) = x_vecs1[i];
    R.col(1) = y_vecs1[i];
    R.col(2) = z_vecs1[i];
    des_gripper_affine1.linear() = R;
    des_gripper_affine1.translation() = tip_origins1[i];
    gripper1_affines.push_back(des_gripper_affine1);

    R.col(0) = x_vecs2[i];
    R.col(1) = y_vecs2[i];
    R.col(2) = z_vecs2[i];
    des_gripper_affine2.linear() = R;
    des_gripper_affine2.translation() = tip_origins2[i];
    gripper2_affines.push_back(des_gripper_affine2);

  }
  ROS_INFO("The circle generation is complete, sending data to Playfile Writer. ");
  data_size = 72;


}

void Trajectory_manipulator::draw_circle_jointspace(){

  double joint_1_0, joint_2_0, joint_3_0, t;
  double joint_1, joint_2;
  double init_radius = 0.5;
  double arrival_time;

  t = 30;
  std::vector <std::vector <double> > data_jsp;
  std::vector <double>  record_jsp(15,0);
  //nposes = data_size_;

  cout << "You will be creating a circle-drawing jointspace playfile \n";

  cout << "The playfile will be consisted of 72 points froming the circle. \n";
  //double PI = 3.14159;
  for (int n = 0; n < 72; n++){
    joint_1 = init_radius*cos(n*2*PI/72);
    joint_2 = init_radius*sin(n*2*PI/72);

    arrival_time = 15 + n * t/72;
    record_jsp[0] = joint_1;
    record_jsp[1] = joint_2;
    record_jsp[14] = arrival_time;

    data_jsp.push_back(record_jsp);
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

		for (int i = 0; i < 14; i++){
			writefile << data_jsp[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data_jsp[n][14] << "\n";
	}

}

/*
REFERENCE
	joint_range_upper_limit:
	1.5994 0.94249 0.24001 3.0485 3.0528 3.0376 3.0399
	joint_range_lower_limit:
	-1.605 -0.93556 -0.002444 -3.0456 -3.0414 -3.0481 -3.0498
*/

void Trajectory_manipulator::sweep_sphere_surface_psm1(){
  //double joint_1_0, joint_2_0, joint_3_0, t;
  double joint_1, joint_2;
  double joint_1_range = 1.5;
  double joint_2_range = 0.9;
  double joint_3 = 0.15;
  double arrival_time = 20;
  double jaw_angle = 0.139; //8 degrees
  //the web will be a 10 x 10 one
  int web_size = 10;
  double sweep_time = 15; // it takes 15 seconds to complete one sweeping
  double creeping_time = 5; // it takes 5 seconds to creep a step
  std::vector <std::vector <double> > data_jsp;
  std::vector <double>  record_jsp(15,0);

  //joint_1 and joint_2 initial poses
  joint_1 = joint_1_range;
  joint_2 = joint_2_range;

  //loading the first position
  record_jsp[0] = joint_1;
  record_jsp[1] = joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);


  //joint_1 sweeps and joint_2 creeps first
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    double delta_2_creeping_step = (2 * joint_2_range) / web_size;

    //step one:
    //joint_1 moves to the other side, joint_2 holds position, assign incremented time for sweeping
    joint_1 = joint_1 * (-1);
    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[0] = joint_1;
    record_jsp[1] = joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_1 holds position, joint_2 creeps a step back position, assign incremented time for creeping
    joint_2 = joint_2 - delta_2_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[0] = joint_1;
    record_jsp[1] = joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //both joints hold still for 2 seconds
  arrival_time = arrival_time + 2;
  //loading exactly the same poses
  record_jsp[0] = joint_1;
  record_jsp[1] = joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //joint_2 sweeps and joint_1 creeps
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    double delta_1_creeping_step = (2 * joint_1_range) / web_size;

    //step one:
    //joint_2 moves to the other side, joint_1 holds position, assign incremented time for sweeping
    joint_2 = joint_2 * (-1);
    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[0] = joint_1;
    record_jsp[1] = joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_2 holds position, joint_1 creeps a step back position, assign incremented time for creeping
    joint_1 = joint_1 - delta_1_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[0] = joint_1;
    record_jsp[1] = joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //both joints hold still for 5 seconds
  arrival_time = arrival_time + 5;
  //loading exactly the same poses
  record_jsp[0] = joint_1;
  record_jsp[1] = joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //writing them into a .jsp file
  std::string oname;
  cout<< "type name: \n";
  cin >> oname;
  const char *cstr = oname.c_str();
  ofstream writefile(cstr);

  ROS_INFO("The circle generation is complete, sending data to embeded Playfile Writer. ");
  data_size = data_jsp.size();

  //ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < data_size; n++){

		for (int i = 0; i < 14; i++){
			writefile << data_jsp[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data_jsp[n][14] << "\n";
	}

}

void Trajectory_manipulator::sweep_sphere_surface_psm2(){
  //double joint_1_0, joint_2_0, joint_3_0, t;
  double joint_1, joint_2;
  double joint_1_range = 1.5;
  double joint_2_range = 0.9;
  double joint_3 = 0.15;
  double jaw_angle = 0.139; //8 degrees
  double arrival_time = 20;
  //the web will be a 10 x 10 one
  int web_size = 10;
  double sweep_time = 15; // it takes 15 seconds to complete one sweeping
  double creeping_time = 5; // it takes 5 seconds to creep a step
  std::vector <std::vector <double> > data_jsp;
  std::vector <double>  record_jsp(15,0);

  //joint_1 and joint_2 initial poses
  joint_1 = joint_1_range;
  joint_2 = joint_2_range;

  //loading the first position
  record_jsp[7] = joint_1;
  record_jsp[8] = joint_2;
  record_jsp[9] = joint_3;
  record_jsp[13] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);


  //joint_1 sweeps and joint_2 creeps first
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    double delta_2_creeping_step = (2 * joint_2_range) / web_size;

    //step one:
    //joint_1 moves to the other side, joint_2 holds position, assign incremented time for sweeping
    joint_1 = joint_1 * (-1);
    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[7] = joint_1;
    record_jsp[8] = joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_1 holds position, joint_2 creeps a step back position, assign incremented time for creeping
    joint_2 = joint_2 - delta_2_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[7] = joint_1;
    record_jsp[8] = joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //both joints hold still for 2 seconds
  arrival_time = arrival_time + 2;
  //loading exactly the same poses
  record_jsp[7] = joint_1;
  record_jsp[8] = joint_2;
  record_jsp[9] = joint_3;
  record_jsp[13] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //joint_2 sweeps and joint_1 creeps
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    double delta_1_creeping_step = (2 * joint_1_range) / web_size;

    //step one:
    //joint_2 moves to the other side, joint_1 holds position, assign incremented time for sweeping
    joint_2 = joint_2 * (-1);
    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[7] = joint_1;
    record_jsp[8] = joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_2 holds position, joint_1 creeps a step back position, assign incremented time for creeping
    joint_1 = joint_1 - delta_1_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[7] = joint_1;
    record_jsp[8] = joint_2;
    record_jsp[9] = joint_3;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //both joints hold still for 5 seconds
  arrival_time = arrival_time + 5;
  //loading exactly the same poses
  record_jsp[7] = joint_1;
  record_jsp[8] = joint_2;
  record_jsp[9] = joint_3;
  record_jsp[13] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //writing them into a .jsp file
  std::string oname;
  cout<< "type name: \n";
  cin >> oname;
  const char *cstr = oname.c_str();
  ofstream writefile(cstr);

  ROS_INFO("The circle generation is complete, sending data to embeded Playfile Writer. ");
  data_size = data_jsp.size();

  //ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < data_size; n++){

		for (int i = 0; i < 14; i++){
			writefile << data_jsp[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data_jsp[n][14] << "\n";
	}


}


// RN
// 27/4/17
// Sphere Rover Restricted Positions

void Trajectory_manipulator::sweep_sphere_surface_psm1_restricted_test(){
  double psm1_joint_1, psm1_joint_2;
  double psm2_joint_1;
  double joint_1_range = 1.5;
  double joint_2_range = 0.9;
  double joint_3 = 0.15;
  double arrival_time = 20;
  double jaw_angle = 0.139; //8 degrees
  // Joint 2 is not restricted by collision avoidance
  double psm1_joint_1_restricted_p = 0.95;
  double psm2_joint_1_evasion_p = 1.5;
  double psm2_joint_1_evasion_n = -0.15;
  double psm_1_max_p;
  // double psm_1_max_n;
  //the web will be a 12 x 12 one
  int web_size = 12;
  double sweep_time = 14; // it takes 14 seconds to complete one sweeping
  double creeping_time = 4; // it takes 4 seconds to creep a step
  std::vector <std::vector <double> > data_jsp;
  std::vector <double>  record_jsp(15,0);
  //data_jsp.clear();
  //record_jsp.clear();

  double delta_2_creeping_step = (2 * joint_2_range) / web_size;
  //joint_1 and joint_2 initial poses
  psm1_joint_1 = psm1_joint_1_restricted_p;
  psm1_joint_2 = joint_2_range;

  psm2_joint_1 = psm2_joint_1_evasion_p;

  //loading the first position
  record_jsp[0] = psm1_joint_1;
  record_jsp[1] = psm1_joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  // PSM2 evasion mode
  record_jsp[7] = psm2_joint_1;
  // record_jsp[8] = joint_2;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //writing them into a .jsp file
  std::string oname;
  cout<< "type name: \n";
  cin >> oname;
  const char *cstr = oname.c_str();
  ofstream writefile(cstr);

  ROS_INFO("The circle generation is complete, sending data to embeded Playfile Writer. ");
  data_size = data_jsp.size();

  //ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < data_size; n++){

		for (int i = 0; i < 14; i++){
			writefile << data_jsp[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data_jsp[n][14] << "\n";
	}

}


void Trajectory_manipulator::sweep_sphere_surface_psm1_restricted(){
  //double joint_1_0, joint_2_0, joint_3_0, t;
  double psm1_joint_1, psm1_joint_2;
  double psm2_joint_1;
  double joint_1_range = 1.5;
  double joint_2_range = 0.9;
  double joint_3 = 0.15;
  double arrival_time = 20;
  double jaw_angle = 0.139; //8 degrees
  // Joint 2 is not restricted by collision avoidance
  double psm1_joint_1_restricted_p = 0.95;
  double psm2_joint_1_evasion_p = 1.5;
  double psm2_joint_1_evasion_n = -0.15;
  double psm1_j1_max_p;
  // double psm_1_max_n;
  //the web will be a 12 x 12 one
  int web_size = 12;
  double sweep_time = 14; // it takes 14 seconds to complete one sweeping
  double creeping_time = 4; // it takes 4 seconds to creep a step
  std::vector <std::vector <double> > data_jsp;
  std::vector <double>  record_jsp(15,0);
  data_jsp.clear();
  //record_jsp.clear();

  double delta_2_creeping_step = (2 * joint_2_range) / web_size;
  //joint_1 and joint_2 initial poses
  psm1_joint_1 = psm1_joint_1_restricted_p;
  psm1_joint_2 = joint_2_range;

  psm2_joint_1 = psm2_joint_1_evasion_p;

  //loading the first position
  record_jsp[0] = psm1_joint_1;
  record_jsp[1] = psm1_joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  // PSM2 evasion mode
  record_jsp[7] = psm2_joint_1;
  // record_jsp[8] = joint_2;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  // If psm1_j1_plus = 1, then PSM1_J1 uses restricted value (1.0), PSM2_J1 uses full range (1.5).
  // If psm1_j1_plus = 0, then PSM1_J1 uses full range, and PSM2_J1 uses evasion value (-.15).
  // The initial position corresponds to psm1_j1_plus = 1.
  int psm1_j1_plus = 1;

  //joint_1 sweeps and joint_2 creeps first
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    //need to decide whehter Joint_2 is greater or smaller than -0.5
    // double delta_2_creeping_step = (2 * joint_2_range) / web_size;

    //step one:
    //joint_1 moves to the other side, joint_2 holds position, assign incremented time for sweeping

    psm1_j1_plus = psm1_j1_plus * (-1); // change J1 direcition DO THIS FIRST!
    // Decide the joint angle based on the arm direcition
    if (psm1_joint_2 > -0.15) {psm1_j1_max_p = psm1_joint_1_restricted_p;}else{psm1_j1_max_p = joint_1_range;}

    if (psm1_j1_plus == 1) {
      psm1_joint_1 = psm1_j1_max_p;
      psm2_joint_1 = psm2_joint_1_evasion_p;
    }else if (psm1_j1_plus == -1){
      psm1_joint_1 = -joint_1_range;
      psm2_joint_1 = psm2_joint_1_evasion_n;
    } //psm1_joint_1 = psm1_joint_1 * (-1);

    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[0] = psm1_joint_1;
    record_jsp[1] = psm1_joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;

    record_jsp[7] = psm2_joint_1 ;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_1 holds position, joint_2 creeps a step back position, assign incremented time for creeping
    psm1_joint_2 = psm1_joint_2 - delta_2_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[0] = psm1_joint_1;
    record_jsp[1] = psm1_joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;

    record_jsp[7] = psm2_joint_1 ;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //re-position PSM1_jpint1
  arrival_time = arrival_time + 4;
  //loading exactly the same poses
  if (psm1_joint_1 > 0){
    record_jsp[0] = psm1_joint_1_restricted_p;
    psm1_joint_1 = psm1_joint_1_restricted_p;}
  if (psm1_joint_1 < 0){
    record_jsp[0] = psm1_joint_1;}
  record_jsp[1] = psm1_joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  record_jsp[7] = psm2_joint_1 ;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //joint_2 sweeps and joint_1 creeps
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    double delta_1_creeping_step = (psm1_joint_1_restricted_p + joint_1_range) / web_size;

    //step one:
    //joint_2 moves to the other side, joint_1 holds position, assign incremented time for sweeping
    // Note that Joint 2 is NOT restricted
    // But Joint 1 +max is restricted to 1.0 (not 1.5 anymore)

    if (psm1_joint_1 > 0) { psm2_joint_1 = psm2_joint_1_evasion_p;}else { psm2_joint_1 = psm2_joint_1_evasion_n;}

    psm1_joint_2 = psm1_joint_2 * (-1);
    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[0] = psm1_joint_1;
    record_jsp[1] = psm1_joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;

    record_jsp[7] = psm2_joint_1 ;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_2 holds position, joint_1 creeps a step back position, assign incremented time for creeping
    psm1_joint_1= psm1_joint_1 - delta_1_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[0] = psm1_joint_1;
    record_jsp[1] = psm1_joint_2;
    record_jsp[2] = joint_3;
    record_jsp[6] = jaw_angle;
    record_jsp[7] = psm2_joint_1 ;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //both joints hold still for 5 seconds
  arrival_time = arrival_time + 5;
  //loading exactly the same poses
  record_jsp[0] = psm1_joint_1;
  record_jsp[1] = psm1_joint_2;
  record_jsp[2] = joint_3;
  record_jsp[6] = jaw_angle;
  record_jsp[7] = psm2_joint_1 ;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //writing them into a .jsp file
  std::string oname;
  cout<< "type name: \n";
  cin >> oname;
  const char *cstr = oname.c_str();
  ofstream writefile(cstr);

  ROS_INFO("The circle generation is complete, sending data to embeded Playfile Writer. ");
  data_size = data_jsp.size();

  //ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < data_size; n++){

		for (int i = 0; i < 14; i++){
			writefile << data_jsp[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data_jsp[n][14] << "\n";
	}

}

void Trajectory_manipulator::sweep_sphere_surface_psm2_restricted(){
  //double joint_1_0, joint_2_0, joint_3_0, t;
  double psm2_joint_1, psm2_joint_2;
  double psm1_joint_1;
  double joint_1_range = 1.5;
  double joint_2_range = 0.9;
  double joint_3 = 0.15;
  double arrival_time = 20;
  double jaw_angle = 0.139; //8 degrees
  // Joint 2 is not restricted by collision avoidance
  double psm2_joint_1_restricted_n = -1.0;
  double psm1_joint_1_evasion_p = 0.15;
  double psm1_joint_1_evasion_n = -1.5;
  double psm2_j1_max_n;
  // double psm_1_max_n;
  //the web will be a 12 x 12 one
  int web_size = 12;
  double sweep_time = 14; // it takes 14 seconds to complete one sweeping
  double creeping_time = 4; // it takes 4 seconds to creep a step
  std::vector <std::vector <double> > data_jsp;
  std::vector <double>  record_jsp(15,0);
  data_jsp.clear();
  //record_jsp.clear();

  double delta_2_creeping_step = (2 * joint_2_range) / web_size;
  //joint_1 and joint_2 initial poses
  psm2_joint_1 = joint_1_range;
  psm2_joint_2 = joint_2_range;

  psm1_joint_1 = psm1_joint_1_evasion_p;

  //loading the first position
  record_jsp[0] = psm1_joint_1;

  record_jsp[7] = psm2_joint_1;
  record_jsp[8] = psm2_joint_2;
  record_jsp[9] = joint_3;
  record_jsp[13] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  // If psm1_j1_plus = 1, then PSM1_J1 uses restricted value (1.0), PSM2_J1 uses full range (1.5).
  // If psm1_j1_plus = 0, then PSM1_J1 uses full range, and PSM2_J1 uses evasion value (-.15).
  // The initial position corresponds to psm1_j1_plus = 1.
  int psm2_j1_plus = 1;

  //joint_1 sweeps and joint_2 creeps first
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    //need to decide whehter Joint_2 is greater or smaller than -0.5
    // double delta_2_creeping_step = (2 * joint_2_range) / web_size;

    //step one:
    //joint_1 moves to the other side, joint_2 holds position, assign incremented time for sweeping

    psm2_j1_plus = psm2_j1_plus * (-1); // change J1 direcition DO THIS FIRST!
    // Decide the joint angle based on the arm direcition
    if (psm2_joint_2 > -0.15) {psm2_j1_max_n = psm2_joint_1_restricted_n;}else{psm2_j1_max_n = -joint_1_range;}

    if (psm2_j1_plus == 1) {
      psm1_joint_1 = psm1_joint_1_evasion_p;
      psm2_joint_1 = joint_1_range;
    }else if (psm2_j1_plus == -1){
      psm1_joint_1 = psm1_joint_1_evasion_n;
      psm2_joint_1 = psm2_j1_max_n;
    } //psm1_joint_1 = psm1_joint_1 * (-1);

    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[0] = psm1_joint_1;

    record_jsp[7] = psm2_joint_1;
    record_jsp[8] = psm2_joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_1 holds position, joint_2 creeps a step back position, assign incremented time for creeping
    psm2_joint_2 = psm2_joint_2 - delta_2_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[0] = psm1_joint_1;

    record_jsp[7] = psm2_joint_1;
    record_jsp[8] = psm2_joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;

    data_jsp.push_back(record_jsp);

  }

  //re-position PSM2_jpint1
  arrival_time = arrival_time + 4;
  //loading exactly the same poses
  if (psm1_joint_1 < 0){
    record_jsp[7] = psm2_joint_1_restricted_n;
    psm1_joint_1 = psm2_joint_1_restricted_n;}
  if (psm1_joint_1 > 0){
    record_jsp[7] = psm2_joint_1;}
    record_jsp[0] = psm1_joint_1;

    //record_jsp[7] = psm2_joint_1;
    record_jsp[8] = psm2_joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;

  data_jsp.push_back(record_jsp);

  //joint_2 sweeps and joint_1 creeps
  for (int n = 0; n < web_size; n++){
    //2 steps in a single loop
    double delta_1_creeping_step = (-psm2_joint_1_restricted_n + joint_1_range) / web_size;

    //step one:
    //joint_2 moves to the other side, joint_1 holds position, assign incremented time for sweeping
    // Note that Joint 2 is NOT restricted
    // But Joint 1 +max is restricted to 1.0 (not 1.5 anymore)

    if (psm2_joint_1 > 0) { psm1_joint_1 = psm1_joint_1_evasion_p;}else { psm1_joint_1 = psm1_joint_1_evasion_n;}

    psm2_joint_2 = psm2_joint_2 * (-1);
    arrival_time =  arrival_time + sweep_time;
    //loading the second target and its arrival_time
    record_jsp[0] = psm1_joint_1;

    record_jsp[7] = psm2_joint_1;
    record_jsp[8] = psm2_joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

    //step two:
    //joint_2 holds position, joint_1 creeps a step back position, assign incremented time for creeping
    psm2_joint_1= psm2_joint_1 - delta_1_creeping_step;
    arrival_time =  arrival_time + creeping_time;
    //loading the third target and its arrival_time
    record_jsp[0] = psm1_joint_1;

    record_jsp[7] = psm2_joint_1;
    record_jsp[8] = psm2_joint_2;
    record_jsp[9] = joint_3;
    record_jsp[13] = jaw_angle;
    record_jsp[14] = arrival_time;
    data_jsp.push_back(record_jsp);

  }

  //both joints hold still for 5 seconds
  arrival_time = arrival_time + 5;
  //loading exactly the same poses
  record_jsp[0] = psm1_joint_1;

  record_jsp[7] = psm2_joint_1;
  record_jsp[8] = psm2_joint_2;
  record_jsp[9] = joint_3;
  record_jsp[13] = jaw_angle;
  record_jsp[14] = arrival_time;
  data_jsp.push_back(record_jsp);

  //writing them into a .jsp file
  std::string oname;
  cout<< "type name: \n";
  cin >> oname;
  const char *cstr = oname.c_str();
  ofstream writefile(cstr);

  ROS_INFO("The circle generation is complete, sending data to embeded Playfile Writer. ");
  data_size = data_jsp.size();

  //ofstream writefile("test_writing_result.psp");
	for (int n = 0; n < data_size; n++){

		for (int i = 0; i < 14; i++){
			writefile << data_jsp[n][i] <<",";
			//writefile << record[i] <<",";
		}

		writefile <<data_jsp[n][14] << "\n";
	}

}



// RN
// 7/5/17
 	
void Trajectory_manipulator::IK_solver_csp_to_jsp(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_)
{
		bool IK_ok = 0;
 		Davinci_IK_solver ik_solver;
		Vectorq7x1 q_vec,q_vec2;
  		std::vector <std::vector <double> > data_jsp;
  		std::vector <double>  record_jsp(15,0);

		for (int i=0;i<data_size_;i++) {
		
		if (ik_solver.ik_solve(gripper1_affines_[i]) && ik_solver.ik_solve(gripper2_affines_[i])){
		//if (ik_solver.ik_solve(gripper1_affines_[i]) == 0){
							IK_ok = 1;
		 					ik_solver.ik_solve(gripper1_affines_[i]); //convert desired pose into equiv joint displacements
		 		       			q_vec = ik_solver.get_soln();
		 		        		ik_solver.ik_solve(gripper2_affines_[i]); //convert desired pose into equiv joint displacements
		 		        		q_vec2 = ik_solver.get_soln();
							cout<<"q_vec: "<<q_vec.transpose()<<endl;
		 					cout<<"q_vec:2 "<<q_vec2.transpose()<<endl;
							record_jsp.clear();
		 					record_jsp.push_back(q_vec[0]);
		 					record_jsp.push_back(q_vec[1]);
		 					record_jsp.push_back(q_vec[2]);
		 					record_jsp.push_back(q_vec[3]);
		 					record_jsp.push_back(q_vec[4]);
		 					record_jsp.push_back(q_vec[5]);
		 					record_jsp.push_back(q_vec[6]);
		 					record_jsp.push_back(q_vec2[0]);
		 					record_jsp.push_back(q_vec2[1]);
		 					record_jsp.push_back(q_vec2[2]);
		 					record_jsp.push_back(q_vec2[3]);
		 					record_jsp.push_back(q_vec2[4]);
		 					record_jsp.push_back(q_vec2[5]);
		 					record_jsp.push_back(q_vec2[6]);
		 					record_jsp.push_back(arrival_times_[i]);
		 					data_jsp.push_back(record_jsp);
		 					record_jsp.clear();
		 					//cout<< "q_vec[0]: " << q_vec[0]<< "\n";
		 					//cout<< "Loader_obj.arrival_times[i]: " << arrival_times_[i] << "\n";
		
	
		 				}else { ROS_WARN("cannot solve IK");
							cout << "  at line: "<< i+1 << "\n";
							IK_ok = 0;
		 					ik_solver.ik_solve(gripper1_affines_[i]); //convert desired pose into equiv joint displacements
		 		       			q_vec = ik_solver.get_soln();
		 		        		ik_solver.ik_solve(gripper2_affines_[i]); //convert desired pose into equiv joint displacements
		 		        		q_vec2 = ik_solver.get_soln();
							cout<<"q_vec: "<<q_vec.transpose()<<endl;
		 					cout<<"q_vec:2 "<<q_vec2.transpose()<<endl;
							record_jsp.clear();
		 					record_jsp.push_back(q_vec[0]);
		 					record_jsp.push_back(q_vec[1]);
		 					record_jsp.push_back(q_vec[2]);
		 					record_jsp.push_back(q_vec[3]);
		 					record_jsp.push_back(q_vec[4]);
		 					record_jsp.push_back(q_vec[5]);
		 					record_jsp.push_back(q_vec[6]);
		 					record_jsp.push_back(q_vec2[0]);
		 					record_jsp.push_back(q_vec2[1]);
		 					record_jsp.push_back(q_vec2[2]);
		 					record_jsp.push_back(q_vec2[3]);
		 					record_jsp.push_back(q_vec2[4]);
		 					record_jsp.push_back(q_vec2[5]);
		 					record_jsp.push_back(q_vec2[6]);
		 					record_jsp.push_back(arrival_times_[i]);
		 					data_jsp.push_back(record_jsp);
		 					record_jsp.clear();
							//break;
							}
			}

			if (IK_ok == 1){
		 		std::string oname_10;
		   		cout<< "type name: \n";
		  		cin >> oname_10;
		   		const char *cstr_10 = oname_10.c_str();
		
		   		ofstream writefile_10(cstr_10);
		 		int data_size_10 =  data_size_;
		 		for (int n = 0; n < data_size_10; n++){
		
		 			for (int i = 0; i < 14; i++){
		 				writefile_10 << data_jsp[n][i] <<",";
		 				//writefile << record[i] <<",";
		 			}
		
		 			writefile_10 <<data_jsp[n][14] << "\n";
		 		}

			}
		 

}


void Trajectory_manipulator::csp_reverse(std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int& data_size_)
{

	ROS_INFO("Calling csp reversing function..");

	Playfile_writer_rn Writer_obj_re;
	std::vector<Eigen::Affine3d> gripper1_affines_re;
	std::vector<Eigen::Affine3d> gripper2_affines_re;
	std::vector<double> gripper_angs1_re;
	std::vector<double> gripper_angs2_re;

	gripper1_affines_re.clear();
	gripper2_affines_re.clear();
	gripper_angs1_re.clear();
	gripper_angs2_re.clear();

	int n = 0;

	for (int i=0;i<data_size_;i++) {
		n = data_size_ - 1 - i;	
		gripper1_affines_re.push_back(gripper1_affines_[n]);
		gripper2_affines_re.push_back(gripper2_affines_[n]);
		gripper_angs1_re.push_back(gripper_angs1_[n]);
		gripper_angs2_re.push_back(gripper_angs2_[n]);		
	}	

	Writer_obj_re.write_playfile(gripper1_affines_re, gripper2_affines_re, arrival_times_, gripper_angs1_re, gripper_angs2_re, data_size_);	 

}



void Trajectory_manipulator::merge_files(std::vector<Eigen::Affine3d> &gripper1_affines_base, 
					std::vector<Eigen::Affine3d> &gripper2_affines_base, 
					std::vector<double> &arrival_times_base, 
					std::vector<double> &gripper_angs1_base, 
					std::vector<double> &gripper_angs2_base, 
					int& data_size_base,
					std::vector<Eigen::Affine3d> &gripper1_affines_, 
					std::vector<Eigen::Affine3d> &gripper2_affines_, 
					std::vector<double> &arrival_times_, 
					std::vector<double> &gripper_angs1_, 
					std::vector<double> &gripper_angs2_, 
					int& data_size_)
{

	ROS_INFO("Calling csp merging function..");

	gripper1_affines_base.insert(gripper1_affines_base.end(), gripper1_affines_.begin(), gripper1_affines_.end());
	gripper2_affines_base.insert(gripper2_affines_base.end(), gripper2_affines_.begin(), gripper2_affines_.end());
	gripper_angs1_base.insert(gripper_angs1_base.end(), gripper_angs1_.begin(), gripper_angs1_.end());
	gripper_angs2_base.insert(gripper_angs2_base.end(), gripper_angs2_.begin(), gripper_angs2_.end());

	for (int i=0;i<data_size_;i++) {

		int increment_level = 0;
		int increment = 0;
		if ((data_size_base-1) < 0) {

			increment = 0;} else {
			increment = arrival_times_base[(data_size_base-1)];
			}

			arrival_times_base.push_back(arrival_times_[i] + increment);	

	}

	data_size_base = data_size_base + data_size_;

}



















