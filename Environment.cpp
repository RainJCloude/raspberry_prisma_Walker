//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

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
 
#include "Actuators.hpp"

#include <fcntl.h>
#include <termios.h>
 
#include <stdio.h>
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>


#define ESC_ASCII_VALUE                 0x1b

using EigenVec=Eigen::Matrix<float, -1, 1>;
/*Tutto si basa sul fatto che gli eigen vector qui li dichiaro come double. Ma la rete nurale vuole per forza float dtype=np.32*/
//#define num_row_in_file 4000
//#define decimal_precision 8
class ENVIRONMENT {

 public:

	explicit ENVIRONMENT(){
		hebiMotor_ = std::make_shared<Actuators>();
		home_path_ = "/home/claudio/raisim_ws/raisimlib";

		float control_dt = 0.01;
		//usando un d gain 0.2 non converge a niente, mettendolo a 2 invece per un po' sembra migliorare ma poi torna a peggiorare
		int num_seq = 3;
   		int num_seq_vel = 2;

		nJointInObsSpace_ = 2;
		double historyPosLength_ = nJointInObsSpace_*num_seq;
		double historyVelLength_ = nJointInObsSpace_*num_seq_vel;
	
		/// MUST BE DONE FOR ALL ENVIRONMENTS
		obDim_ = historyPosLength_ + historyVelLength_ + 3;
		int privileged_obs_dim = 6 + 3 + 2;

		actionDim_ = nJoints_; 
		actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_); linkTorque_.setZero(actionDim_);
		if(use_privileged_){
			obDim_ += privileged_obs_dim;
		}
		obDouble_.setZero(obDim_);

		/// action scaling
		double action_std = 0.1;
		actionStd_.setConstant(action_std);

 		m1_pos_.setZero(traj_size);
		m2_pos_.setZero(traj_size); 

		motors = new Actuators();
		motors->initHandlersAndGroup(ActuatorConnected_, num_seq, num_seq_vel);
		openFile();
		index_imitation_ = 0;
	}

	void openFile(){

		std::fstream m1_traj;
    		std::fstream m2_traj;

		m1_traj.open(home_path_ + "/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/pos_m1_18s.txt", std::ios::in);
    		m2_traj.open(home_path_ + "/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/pos_m2_18s.txt", std::ios::in);
	
		Eigen::VectorXd m1_pos(traj_size);
		Eigen::VectorXd m2_pos(traj_size);

		if(m1_traj.is_open() && m2_traj.is_open()){
			for(int j = 0;j<traj_size;j++){
				m1_traj >> m1_pos(j);  //one character at time store the content of the file inside the vector

				m1_pos_=-m1_pos;
				m1_stdvector_.push_back(m1_pos(j));

				m2_traj >> m2_pos(j);

				m2_pos_=-m2_pos;
				m2_stdvector_.push_back(m2_pos(j));

			}
		}
		else
			std::cout<<"File not opend!"<<std::endl;
		
		m1_traj.close(); 
		m2_traj.close();
	}

	void reset(){}

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
 
	void step(const Eigen::Ref<EigenVec>& action) {
		/// action scaling

		pTarget3_ = action.cast<double>(); //casting function to double from numpy
		pTarget3_ = pTarget3_.cwiseProduct(actionStd_);
		actionMean_ << m1_pos_(index_imitation_), m2_pos_(index_imitation_), 0.0;
		pTarget3_ += actionMean_;
		current_action_ = pTarget3_;

		//pTarget3_.tail(nJoints_) << std::sin(2*M_PI*5000*t), std::sin(2*M_PI*5000*t + 0.25), 0;
		//t+=control_dt_;

		if(ActuatorConnected_){
			bool commandInPosition = true;
			if(commandInPosition)
			//Commanded with controlStrategy2
				motors->sendCommand(pTarget3_.tail(nJoints_), true);
			else
				motors->sendCommand(linkTorque_.tail(nJoints_), false);
		}
		
		updateObservation();
		incrementIndices();
		
	}

	int num_obs(){
		return obDim_;
	}
	
	int num_actions(){
		return actionDim_;
	}

	void incrementIndices(){

		index_imitation_++;

		if(index_imitation_ >= traj_size){
			index_imitation_ = 0;
		}
	}
	void updateObservation() {
		
		Eigen::VectorXd obs_motors = motors->getFeedback(use_privileged_, m1_pos_(index_imitation_), m2_pos_(index_imitation_));
 		obDouble_ << obs_motors, 
		current_action_;

	}


	//VectorXd is a double type. You cannot cast to float otherwise you get errors. In raisim they used EigenVec as template 
	//to perform casting. Eigen::Ref is used to pass an eigen value as reference
	void observe(Eigen::Ref<EigenVec> ob) {
		ob = obDouble_.cast<float>();
	}

	
public:
	Eigen::VectorXd ob_;
private:
	int actionDim_ = 0;
	std::string home_path_;
	int nJoints_ = 3;
	int obDim_ = 0;
	Eigen::VectorXd m1_pos_;
	Eigen::VectorXd m2_pos_;
	Eigen::Vector3d current_action_;
	Eigen::VectorXd actionMean_, actionStd_, obDouble_;
	
	Actuators *motors;
	std::shared_ptr<Actuators> hebiMotor_;

	const int traj_size = 1818;
	bool ActuatorConnected_ = false;

	std::ofstream torques;

	std::vector<double> m1_stdvector_;
	std::vector<double> m2_stdvector_;
	
	int nJointInObsSpace_ = 0;
	Eigen::Vector2d direction_;
	Eigen::VectorXd linkTorque_;
	bool use_privileged_ = false;

protected:
	int index_imitation_;
	Eigen::Vector3d pTarget3_ = Eigen::Vector3d::Zero();

};
//thread_local std::mt19937 raisim::ENVIRONMENT::gen_;



namespace py = pybind11;

PYBIND11_MODULE(prisma_walker, m) { //the name here must match the name of the module, otherwise pyhton cannot see it
	py::class_<ENVIRONMENT>(m, "ENVIRONMENT")
    	.def(pybind11::init<>())
	.def("reset", &ENVIRONMENT::reset)
	.def("step", &ENVIRONMENT::step)
	.def("observe", &ENVIRONMENT::observe)
	.def("num_obs", &ENVIRONMENT::num_obs)
	.def("num_actions", &ENVIRONMENT::num_actions)	
	.def("getReference", &ENVIRONMENT::getReference)
	.def("getJointPositions", &ENVIRONMENT::getJointPositions)
	.def("getJointVelocities", &ENVIRONMENT::getJointVelocities);
}
