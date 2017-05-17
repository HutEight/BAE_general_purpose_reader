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
void Trajectory_manipulator::draw_cycle(){


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
