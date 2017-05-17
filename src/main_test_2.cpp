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
#include <general_purpose_reader/trajectory_manipulator.h>

#include <davinci_kinematics/davinci_joint_publisher.h>
#include <davinci_kinematics/davinci_kinematics.h>

using namespace std;
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
// ofstream just_a_bloody_stream("temp_ttp.jsp");
bool setFlag() {
cout<<"\n\033[1;37mDo you wish to Proceed and Select Mode (press 1) or Restart (press 2) or \033[1;31mQuit (press 0)?\033[0m\n";
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
	ros::init(argc, argv, "main_test_2");
	ros::NodeHandle nh;

	cout << "\n\033[1;34mHit ENTER to Start...\033[0m\n\n";

//int en = 1;
while (ros::ok()){
//First enter and store a list of playfile names
	cin.clear();
	cin.ignore(numeric_limits<streamsize>::max(), '\n');
	cout << "\033[1;31mGENERAL PURPOSE PLAYFILE READER MKI\033[0m\n";
	cout << "BAE SYSTEMS" << endl << endl;
	
	cout << "If you want to work on one or multiple \033[1;34mCartesian space playfiles\033[0m, please enter the names, and hit Enter to finish." << endl << endl
	<< "   * The utilities that require at least one playfile name:" << endl
	<< "     \033[1;34mGearbox\033[0m" << endl
	<< "     \033[1;34mGearbox SMART\033[0m" << endl
	<< "     \033[1;34mTime Interpolation\033[0m" << endl
	<< "     \033[1;34mPlayfile split\033[0m" << endl
	<< "     \033[1;34mIK solution\033[0m" << endl 
	<< "     \033[1;34mCartesian playfile reverse\033[0m" << endl << endl
	<< "If you don't need to input a playfile, please just hit the Entre to skip." << endl << endl
	<< "   * The utilities that does not require any playfile names:" << endl 
	<< "     ..List not complete" << endl << endl
	<< "\033[1;31mTo QUIT\033[0m, hit Enter to skip first and then select."<< endl << "Do not try 'Ctrl + C'.. " << endl << endl;
	cout << "* * * Now please type in the playfile names or just hit Enter to skip * * *"<<"\n";

	std::string templine = "not_empty";

	int n_entries = 0;
	vector<std::string> name_list;
	name_list.clear();


	while (templine.length() != 0) {
		cout << "Enter a playfile name: \n";
		getline(cin, templine);
		name_list.push_back(templine); //name_list
		stringstream streamname(templine);
		n_entries++;
		if (templine.length() != 0){
			cout << "Loading.. entry #"<< n_entries << ": "<< templine << "\n"<< "\n";
		}
	}

	cout << "	\033[1;32mInput terminated..\033[0m\n";
	cout << "	You have entered "<< --n_entries << " files. \n";

//Then choose 1. Gearbox, 2. Playfile Merger


	int mode_code;

	while (setFlag()){
	// ROS_INFO("Filelist:");
	cout << "\033[1;32mFILE LIST:\033[0m\n"
	<< "Plase notice that the list number starts from 0" << endl;

	Playfile_loader Loader_obj;	//will not be put into switch case

	Playfile_writer_rn Writer_obj;
	Playfile_gearbox Gearbox_obj;
	Trajectory_manipulator Manipulator_obj;

	for (int i=0; i < n_entries; i++){
		cout << i << ". "<< name_list[i]<< "\n";
		}

	cout << "\n";

	   cout << "\033[1;37mPlease Select Mode:\033[0m " << endl
		<< "Press 1 for Gearbox" << endl
		<< "Press 2 for Gearbox SMART" << endl
		<< "Press 3 for Time Interpolation" << endl
		<< "Press 4 to Move the gripper (NOT TESTED)" << endl
		<< "Press 5 to Generate a playfile to command the gripper to draw a circle (NOT TESTED)" << endl
		<< "Press 6 to Split a playfile" << endl
		<< "Press 7 to Generate a cirle drawing playfile in jointspace" << endl
		<< "Press 8 to Generate 2 sphere-rover playfiles for both psms in jointspace" << endl
		<< "Press 10 to Solve IK for a csp file" << endl
		<< "Press 11 to Generate 2 sphere-rover (collision avoidance) playfiles for both psms in jointspace (for the current da Vinci config ONLY)" << endl
		<< "Press 12 to Reverse a csp file" << endl 
		<< "Press 13 to Merge csp files" << endl << endl
		<< "\033[1;31mPress 9 to change mode / return\033[0m \n"
		<< "\033[1;31mPress 999 to quit node:\033[0m \n" << endl;
	


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


		case 2://Gearbox Smart
			{ ROS_INFO("Gearbox SMART");
			n_entries++;
			ROS_INFO("Choose one file to scale:");
			int choice2;
			cin >> choice2;
			Loader_obj.load_data(name_list[choice2]);
			cout << "Please enter scale: x";
			double scale2;

			cin >> scale2;
			Gearbox_obj.change_gear_smart(scale2, Loader_obj.arrival_times, Loader_obj.data_size);

			Writer_obj.write_playfile(Loader_obj.gripper1_affines, Loader_obj.gripper2_affines, Gearbox_obj.new_arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);

			name_list.push_back(Writer_obj.oname);
			//cout << Writer_obj.oname << "\n";
			ROS_INFO("New playfile saved!");
			}
		break;


		case 3://Interpolate extra time between lines
			ROS_INFO("Ready to interpolate time.");
			ROS_INFO("Choose one file to interpolate time:");
			double length;
			int line_num;
			n_entries++;
			int choice3;
			cin >> choice3;
			Loader_obj.load_data(name_list[choice3]);

			//bool interpolate (int& line_, double& interval_, std::vector<Eigen::Affine3d> &gripper1_affines_, std::vector<Eigen::Affine3d> &gripper2_affines_, std::vector<double> &arrival_times_, std::vector<double> &gripper_angs1_, std::vector<double> &gripper_angs2_, int data_size_);
			Gearbox_obj.interpolate(Loader_obj.gripper1_affines, Loader_obj.gripper2_affines, Loader_obj.arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);
			Writer_obj.write_playfile(Gearbox_obj.gripper1_affines,Gearbox_obj.gripper2_affines, Gearbox_obj.new_arrival_times, Gearbox_obj.gripper_angs1, Gearbox_obj.gripper_angs2,Gearbox_obj.data_size);

			name_list.push_back(Writer_obj.oname);
			//cout << Writer_obj.oname << "\n";
			ROS_INFO("New playfile saved!");
			//ROS_INFO("MARKER 2");
		break;


		case 4: //Move the gripper
			ROS_INFO("Choose one file to manipulate:");
			int choice4;
			cin >> choice4;
			Loader_obj.load_data(name_list[choice4]);
			Manipulator_obj.move_gripper_tip(Loader_obj.tip_origins1, Loader_obj.tip_origins2, Loader_obj.nposes);
			Loader_obj.update_origins(Manipulator_obj.new_tip_origins1, Manipulator_obj.new_tip_origins2);
			Writer_obj.write_playfile(Loader_obj.gripper1_affines,Loader_obj.gripper2_affines, Loader_obj.arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);
		break;

		case 5://Create a playfile to draw a circle
			Manipulator_obj.draw_circle ();
			Writer_obj.write_playfile(Manipulator_obj.gripper1_affines,Manipulator_obj.gripper2_affines,Manipulator_obj.arrival_times,Manipulator_obj.gripper_angs1,Manipulator_obj.gripper_angs2,Manipulator_obj.data_size);
		break;


		case 6://Split a playfile into two
			ROS_INFO("Choose one file to split:");
			int choice6;
			cin >> choice6;
			Loader_obj.load_data(name_list[choice6]);
			Writer_obj.split_playfile(Loader_obj.gripper1_affines,Loader_obj.gripper2_affines,Loader_obj.arrival_times,Loader_obj.gripper_angs1,Loader_obj.gripper_angs2,Loader_obj.data_size);
		break;

		case 7://Draw circle jsp
			ROS_INFO("Drawing a circle in jsp");
			Manipulator_obj.draw_circle_jointspace ();
		break;

   		case 8://Sphere surface sweep
			ROS_INFO("creating a psm1 sphere surface rover jsp");
			Manipulator_obj.sweep_sphere_surface_psm1();
			ROS_INFO("creating a psm2 sphere surface rover jsp");
			Manipulator_obj.sweep_sphere_surface_psm2();
		break;


		case 9://Null, RETURN
			// ROS_WARN("Returning..");
			cout << "\033[1;31mReturning..\033[0m\n";
		break;


		case 999:
			cout << "	\033[1;31mForce Quit..\033[0m\n";
			exit(0);
		break;


		case 11://Sphere surface sweep (collision avoidance)
			ROS_INFO("creating a psm1 sphere surface rover (collision avoidance) jsp");
			Manipulator_obj.sweep_sphere_surface_psm1_restricted();
			ROS_INFO("creating a psm2 sphere surface rover (collision avoidance) jsp");
			Manipulator_obj.sweep_sphere_surface_psm2_restricted();
			// Debug use
			// ROS_INFO("creating a psm1 sphere surface rover test jsp");
			// Manipulator_obj.sweep_sphere_surface_psm1_restricted_test();
			//ROS_INFO("creating a psm2 sphere surface rover jsp");
			//Manipulator_obj.sweep_sphere_surface_psm2();
		break;

		// DO NOT PUT ANY NEW CASES AFTER THIS line
		// CASE 11 MUST BE THE LAST ONE IN THE SEQUENCE

		case 10://IK solver
		//
		 	ROS_INFO("Choose one file to solve IK:");
		 	int choice10;
		 	cin >> choice10;
		 	Loader_obj.load_data(name_list[choice10]);
		 	cout << "Solving IK .. \n";
			Manipulator_obj.IK_solver_csp_to_jsp(Loader_obj.gripper1_affines,Loader_obj.gripper2_affines, Loader_obj.arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);
			
			
			
		break;

		case 12: // CSP reverse
			cout << "\n\033[1;34mChoose one file to reverse:\033[0m\n";
			int choice12;
			cin >> choice12;
			Loader_obj.load_data(name_list[choice12]);
			Manipulator_obj.csp_reverse(Loader_obj.gripper1_affines,Loader_obj.gripper2_affines, Loader_obj.arrival_times, Loader_obj.gripper_angs1, Loader_obj.gripper_angs2,Loader_obj.data_size);
			
		break;

		case 13: // Merge files
			bool choose_all = false;
			int n_13 = 0;
			cout << "\n\033[1;34mDo you want to merge all the input files into one? Please press 1\033[0m" << endl
			<< "If not, please press 0 and select the files one by one in your preferable order:" << endl;
			cin >> choose_all;


			std::vector<Eigen::Affine3d> gripper1_affines_base;
			std::vector<Eigen::Affine3d> gripper2_affines_base; 
			std::vector<double> arrival_times_base; 
			std::vector<double> gripper_angs1_base;
			std::vector<double> gripper_angs2_base; 
			int data_size_base;
			std::vector<Eigen::Affine3d> gripper1_affines_; 
			std::vector<Eigen::Affine3d> gripper2_affines_; 
			std::vector<double> arrival_times_; 
			std::vector<double> gripper_angs1_; 
			std::vector<double> gripper_angs2_; 
			int data_size_;
			// TODO really can create a container object to do the above

			gripper1_affines_base.clear();
			gripper2_affines_base.clear(); 
			arrival_times_base.clear(); 
			gripper_angs1_base.clear();
			gripper_angs2_base.clear(); 
			data_size_base = 0;

			gripper1_affines_.clear(); 
			gripper2_affines_.clear(); 
			arrival_times_.clear(); 
			gripper_angs1_.clear(); 
			gripper_angs2_.clear(); 
			
			if (choose_all == true){
				n_13 = n_entries;

					for (int i = 0; i < (n_13); i++) {

						Playfile_loader Loader_obj_2;
						Loader_obj_2.load_data(name_list[i]);
					
						gripper1_affines_ = Loader_obj_2.gripper1_affines;
						gripper2_affines_ = Loader_obj_2.gripper2_affines;
						arrival_times_ = Loader_obj_2.arrival_times;
						gripper_angs1_ = Loader_obj_2.gripper_angs1;
						gripper_angs2_ = Loader_obj_2.gripper_angs2;
						data_size_ = Loader_obj_2.data_size;

						// base will be changed
						Manipulator_obj.merge_files(gripper1_affines_base, 
								gripper2_affines_base, 
								arrival_times_base, 
								gripper_angs1_base, 
								gripper_angs2_base, 
								data_size_base,
								gripper1_affines_, 
								gripper2_affines_, 
								arrival_times_, 
								gripper_angs1_, 
								gripper_angs2_, 
								data_size_);

						gripper1_affines_.clear(); 
						gripper2_affines_.clear(); 
						arrival_times_.clear(); 
						gripper_angs1_.clear(); 
						gripper_angs2_.clear(); 
	
					}

			Writer_obj.write_playfile(gripper1_affines_base, 
							gripper2_affines_base, 
							arrival_times_base, 
							gripper_angs1_base, 
							gripper_angs2_base, 
							data_size_base);
		
			
			} else { ROS_WARN("THIS UTILITY IS NOT COMPLETE");
				
				
				
			}



		break;



	}
	}







}
}
