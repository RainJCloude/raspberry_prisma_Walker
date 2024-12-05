#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "command.hpp"
#include "/usr/local/include/eigen3/Eigen/Dense"
#include <iostream>
#include <stdio.h>
#include <fstream>

// Control table address
constexpr int addrTorqueEnable = 64;            // Control table address is different in Dynamixel model
constexpr int addrGoalPosition = 116;
constexpr int addrPresentPosition = 132;
constexpr int addrPresentVelocity = 128;

// Protocol version
constexpr int protocolVersion = 2.0;            // See which protocol version is used in the Dynamixel

// Default setting
constexpr int dxl_id = 1;  // Dynamixel ID: 1
constexpr int baudrate = 2000000;
#define deviceName "/dev/ttyUSB0"     // Check which port is being used on your controller

constexpr int TorqueEnable = 1;                 		// Value for enabling the torque
constexpr int TorqueDisable = 0;                  		// Value for disabling the torque
constexpr int dxlMinimumPositionValue = 0;          	// Dynamixel will rotate between this value
constexpr int dxlMaximumPositionValue = 4095;      		// and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
constexpr int dxlMovingStatusThreshold = 20;            // Dynamixel moving status threshold


class Actuators {

 public:

Actuators(){
		

	group_ = lookup_.getGroupFromNames({"X5-4"}, {"X-01059", "X-01077"} );

	if (!group_){
		std::cout << "No group found!"<<std::endl;
	}
	else{
		num_modules_ = group_->size();
		std::cout<<"numero moduli: "<<num_modules_<<std::endl;

		group_->setCommandLifetimeMs(10); //0.01s
		group_->setFeedbackFrequencyHz(100);  
	}
}

~Actuators(){};

void sendCommand(Eigen::VectorXd torqueOrPositionCommand){

	cmd_.setPosition(torqueOrPositionCommand.head(2));

 	group_->sendCommand(cmd_);

}


void getMotorPos(const hebi::GroupFeedback& feedback,  Eigen::Vector3d & motor_pos){
	double dxl_pos;
	const double m1_pos_fbk = feedback[0].actuator().position().get();
	const double m2_pos_fbk = feedback[1].actuator().position().get();
	
	motor_pos << m1_pos_fbk, m2_pos_fbk, dxl_pos;
}

void getMotorVel(const hebi::GroupFeedback& feedback, Eigen::Vector3d & motor_vel){
	double dxl_vel;
	const double m1_vel_fbk = feedback[0].actuator().velocity().get();
	const double m2_vel_fbk = feedback[1].actuator().velocity().get();

	motor_vel << m1_vel_fbk, m2_vel_fbk, dxl_vel;
}


/*void writeDataOnFiles(
	const std::vector<double>& m1_pos, 
	const std::vector<double>& m2_pos,
	const std::vector<double>& m1_vel, 
	const std::vector<double>& m2_vel,
	const std::vector<double>& m1_torque, 
	const std::vector<double>& m2_torque
)
{

	std::fstream m1_positions;
	std::fstream m2_positions;

	m1_positions.open("m1_positions.txt", std::ios::out);
	m2_positions.open("m2_positions.txt", std::ios::out);

	m1_traj << m1;
	m2_traj << m2;
}*/


void writeDataOnFiles(
	const std::vector<double>& m1, 
	const std::vector<double>& m2,
	std::string file1,
	std::string file2
)
{

	std::fstream m1_positions;
	std::fstream m2_positions;

	m1_positions.open(file1, std::ios::out);
	m2_positions.open(file2, std::ios::out);

	for (const int &value : m1) {
        m1_positions << value << std::endl; 
    	}
	m1_positions.close();

	for (const int &value : m2) {
        m2_positions << value << std::endl; 
    	}
	m2_positions.close();
}


void getFeedback(){

	group_->sendFeedbackRequest();
	group_->getNextFeedback(Gfeedback_);
	

	const double m1_pos_fbk = Gfeedback_[0].actuator().position().get();
	m1_pos.push_back(m1_pos_fbk);
	const double m2_pos_fbk = Gfeedback_[1].actuator().position().get();
	m2_pos.push_back(m2_pos_fbk);

	const double m1_vel_fbk = Gfeedback_[0].actuator().velocity().get();
	m1_vel.push_back(m1_vel_fbk);
	const double m2_vel_fbk = Gfeedback_[1].actuator().velocity().get();
	m2_vel.push_back(m2_vel_fbk);

	const double m1_torque_fbk = Gfeedback_[0].actuator().effort().get();
	m1_torque.push_back(m1_torque_fbk);
	const double m2_torque_fbk = Gfeedback_[1].actuator().effort().get();
	m2_torque.push_back(m2_torque_fbk);	

	//writeDataOnFiles(m1_pos, m2_pos, m1_vel, m2_vel, m1_torque, m2_torque);
	
}     


private:

//hebi
bool ActuatorConnected_ = true;
int num_modules_;
hebi::Lookup lookup_;
std::shared_ptr<hebi::Group> group_;
hebi::GroupCommand cmd_ = hebi::GroupCommand(2);
hebi::GroupFeedback Gfeedback_ = hebi::GroupFeedback(2);
hebi::Command::ControlStrategy control_strategy;  //ControlStrategy it's an enum



std::vector<double> m1_pos;
std::vector<double> m2_pos;

std::vector<double> m1_vel;
std::vector<double> m2_vel;

std::vector<double> m1_torque;
std::vector<double> m2_torque;

};




int main(){

	Actuators hebi;

	while(1){
		hebi.getFeedback();
	}

	hebi.writeDataOnFiles(hebi.m1_pos, hebi.m2_pos, "m1_positions", "m2_positions");
	hebi.writeDataOnFiles(hebi.m1_vel, hebi.m2_vel, "m1_velocities", "m2_velocities");
	hebi.writeDataOnFiles(hebi.m1_torque, hebi.m2_torque, "m1_torques", "m2_torques");

	return 1;
}