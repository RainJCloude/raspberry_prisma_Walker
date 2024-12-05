#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <iostream>
#include <fstream>
#include "/usr/include/eigen3/Eigen/Dense"
#include <kdl_parser/kdl_parser.hpp> //ros humble has this path

using namespace std;


bool loadURDF_into_robot_desc_string(const std::string& urdf_path, std::string& robot_desc_string) {
    std::ifstream file;
    file.open(urdf_path.c_str());
    /*std::string a;
    file>>a;
    file>>a; //string after string you put the code inside the 
    std::cout<<a<<std::endl;*/
    if (!file) {
        cerr << "Error: Could not open URDF file." << endl;
        return false;
    }
    std::stringstream buffer;
    buffer << file.rdbuf(); //return a pointer to an object stringsbuffer
    robot_desc_string = buffer.str();
    return true;
}



	void openFile(std::vector<double> &m1_pos_vector, std::vector<double> & m2_pos_vector){

		std::fstream m1_traj;
    	std::fstream m2_traj;

        std::string home_path_ = "/home/claudio/raisim_ws/raisimlib";

		m1_traj.open(home_path_ + "/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/pos_m1_18s.txt", std::ios::in);
    	m2_traj.open(home_path_ + "/raisimGymTorch/raisimGymTorch/env/envs/prisma_walker/pos_m2_18s.txt", std::ios::in);

        double m1_pos;
        double m2_pos;

		if(m1_traj.is_open() && m2_traj.is_open()){
			for(int j = 0;j<1818;j++){
				m1_traj >> m1_pos; 
                m1_pos_vector.push_back(m1_pos);
				m2_traj >> m2_pos;
                m2_pos_vector.push_back(m2_pos);
			}
		}
		else
			std::cout<<"File not opend!"<<std::endl;
		
		m1_traj.close(); 
		m2_traj.close();

	}



int main() {
    // Define the chain
    KDL::Chain chain;

    std::string robot_desc_string;
    std::string urdf_path = "/home/claudio/raisim_ws/raisimlib/rsc/prisma_walker/urdf/prisma_walker.urdf";
    if(!loadURDF_into_robot_desc_string(urdf_path, robot_desc_string))
        return -1;

    KDL::Tree prismaWalker_tree;
    if (!kdl_parser::treeFromString(robot_desc_string, prismaWalker_tree)) {
        cerr << "Failed to construct KDL tree from URDF." << endl;
        return -1;
    }

    std::vector<double> m1_pos_vector;
    std::vector<double> m2_pos_vector;
    openFile(m1_pos_vector, m2_pos_vector);

    KDL::SegmentMap segment_map = prismaWalker_tree.getSegments();
    KDL::SegmentMap::const_iterator it = segment_map.find("base_link");   

	std::string base_link = "base_link"; 
	std::string foot_link = "piede_interno";
	if ( !prismaWalker_tree.getChain(base_link, foot_link, chain) ) 
        return false; 

    // Create a solver based on the chain
    KDL::ChainFkSolverPos_recursive *fksolver = new KDL::ChainFkSolverPos_recursive(chain);
    // Define the joint positions (angles)
    KDL::JntArray* joint_positions = new KDL::JntArray(chain.getNrOfJoints() );

    std::cout<<"number of joints: "<<chain.getNrOfJoints()<<std::endl;
    std::cout<<"number of links: "<<chain.getNrOfSegments()<<std::endl;

    int num_joints = chain.getNrOfJoints();
	KDL::Frame end_effector_pose;

    std::fstream posa_x;
    std::fstream posa_y;
    std::fstream posa_z;
    posa_x.open("posa_x.txt", std::ios::out);
    posa_y.open("posa_y.txt", std::ios::out);
    posa_z.open("posa_z.txt", std::ios::out);
    for(int j = 0; j < m1_pos_vector.size(); j++){
        
        joint_positions->data[0] = m1_pos_vector[j];
        joint_positions->data[1] = m2_pos_vector[j];

        // Define a frame to hold the result
        

        // Calculate forward kinematics
        fksolver->JntToCart(*joint_positions, end_effector_pose);

      
        posa_x << end_effector_pose.p.x()<<std::endl;
        posa_y << end_effector_pose.p.y()<<std::endl;
        posa_z << end_effector_pose.p.z()<<std::endl;


    }
  
    return 0;
}
