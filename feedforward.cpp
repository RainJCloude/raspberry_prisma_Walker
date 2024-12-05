//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//
#include <stdlib.h>
#include <set>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include "Eigen/Eigen"

#include <cstdlib>
#include <fstream>
#include <vector> 
#include <csignal>

#include "Actuators.hpp"

#include <fcntl.h>
#include <termios.h>
 
#include <stdio.h>

#define ESC_ASCII_VALUE                 0x1b

using EigenVec=Eigen::Matrix<float, -1, 1>;
/*Tutto si basa sul fatto che gli eigen vector qui li dichiaro come double. Ma la rete nurale vuole per forza float dtype=np.32*/
//#define num_row_in_file 4000
//#define decimal_precision 8

bool g_running = true;


class ENVIRONMENT {

 public:

	explicit ENVIRONMENT(){
		hebiMotor_ = std::make_shared<Actuators>();

		float control_dt = 0.01;
 
		motors = new Actuators();
		motors->initHandlersAndGroup(ActuatorConnected_, 3, 3);
		openFile();
		index_imitation_ = 0;
	}

	void openFile(){

		std::fstream m1_traj;
    		std::fstream m2_traj;

		m1_traj.open("m1_positions.txt", std::ios::in);
    		m2_traj.open("m2_positions.txt", std::ios::in);


            double value1, value2;
		while(m1_traj >> value1 && m2_traj >> value2){
			m1_pos_.push_back(value1);
                  m2_pos_.push_back(value2); 
		}
		
		m1_traj.close(); 
		m2_traj.close();
	}

	void getReference(Eigen::Ref<EigenVec>& tau){
		tau = pTarget3_.cast<float>();
	}

	void getJointPositions(Eigen::Ref<EigenVec>& q){
		Eigen::VectorXd motorVariables;
		motorVariables.setZero(6);

		motorVariables = motors->dataPlotMotorVariables();
		q = motorVariables.segment(0,3).cast<float>();
	}
	
	void getJointVelocities(Eigen::Ref<EigenVec>& dotq){
		Eigen::VectorXd motorVariables;
		motorVariables.setZero(6);
		motorVariables = motors->dataPlotMotorVariables();
		dotq = motorVariables.segment(3,3).cast<float>();
	}
 
	void step() {
            
            while(g_running){
                  pTarget3_ << m1_pos_[index_imitation_], m2_pos_[index_imitation_], 0.0;
      
                  motors->sendCommand(pTarget3_.tail(3), true);

                  incrementIndices();		
            }
	}


	void incrementIndices(){

		index_imitation_++;

		if(index_imitation_ >= traj_size){
			index_imitation_ = 0;
		}
	}


      static void signalHandler(int signal) {  //SIGINT is ctrl-c
		if (signal == SIGINT) {
			std::cout << "\nCtrl+C pressed. Exiting gracefully..." << std::endl;
			g_running = false; // Signal the thread to stop
			//pthread_cancel(workerThread); // Cancel the thread (optional)
		}
	}
	
public:
	Eigen::VectorXd ob_;
private:
	int actionDim_ = 0;
	std::string home_path_;
 
      std::vector<double> m1_pos_;
      std::vector<double> m2_pos_;

	Actuators *motors;
	std::shared_ptr<Actuators> hebiMotor_;

	const int traj_size = 1818;
	bool ActuatorConnected_ = false;

	std::ofstream torques;

	std::vector<double> m1_stdvector_;
	std::vector<double> m2_stdvector_;
 

protected:
	int index_imitation_;
	Eigen::Vector3d pTarget3_ = Eigen::Vector3d::Zero();

};



int main(){

      ENVIRONMENT e;
      std::signal(SIGINT, ENVIRONMENT::signalHandler);

      e.step();


      return 1;
}
//thread_local std::mt19937 raisim::ENVIRONMENT::gen_;



