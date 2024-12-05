#include <stdio.h>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "lookup.hpp"
#include "group_command.hpp"
#include "group_feedback.hpp"
#include "command.hpp"

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"

using std::placeholders::_1;


class Monopod : public rclcpp::Node
{
public:
  Monopod() : Node("monopod_robot")
  {
    // Subscribe to URDF
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1))
        .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&Monopod::robotDescriptionCallback, this, _1));

    this->declare_parameter("ang_value", 0.0);
    double ang_ref = this->get_parameter("ang_value").as_double();

	  group_ = lookup_.getGroupFromNames({"X5-4"}, {"X-01059", "X-01077"} );
    if(!group_){
      std::cout << "No group found!"<<std::endl;
      return -1;
    }
		group_->setFeedbackFrequencyHz(100); 
    quat_.setZero(4);

    publisher_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
      "set_position", 
      10
    );

    auto joint_state_msg = dynamixel_sdk_custom_interfaces::msg::SetPosition();
    double j1 = 0.0;

  
    joint_state_msg.id = 11;
    joint_state_msg.position = convert_to_bits(q_init(i));
    publisher_->publish(joint_state_msg);
    std::cout << "Joint " << i << " set to " << convert_to_bits(q_init(i)) << std::endl;
 
    while(rclcpp::ok()){
      
    }
  }



  Eigen::VectorXd Monopod::getFeedback(const bool usePrivileged, const double m1Pos, const double m2Pos){

    group_->sendFeedbackRequest();
    group_->getNextFeedback(Gfeedback_);
    
    double dxl_vel;
    const double m1_vel_fbk = Gfeedback_[0].actuator().velocity().get();
    const double m2_vel_fbk = Gfeedback_[1].actuator().velocity().get();
    //packetHandler_->read4ByteTxRx(portHandler_, dxl_id, addrPresentVelocity, (uint32_t*)&dxl_vel, &dxl_error_);

    double dxl_pos; 
    const double m1_pos_fbk = Gfeedback_[0].actuator().position().get();
    const double m2_pos_fbk = Gfeedback_[1].actuator().position().get();
    //packetHandler_->read4ByteTxRx(portHandler_, dxl_id, addrPresentPosition, (uint32_t*)&dxl_pos, &dxl_error_);
  
    //Orientation and body velocity got from the first joint
    const auto real_orientation_ = Gfeedback_[0].imu().orientation().get();
    const auto imu_ang_vel = Gfeedback_[0].imu().gyro().get();

    quat_[0] = real_orientation_.getW();
    quat_[1] = real_orientation_.getX();
    quat_[2] = real_orientation_.getY();
    quat_[3] = real_orientation_.getZ();

    rot_ = QuatToMat(quat_);
    swapMatrixRows(rot_); //raisim had a different way

    //R0 is the base orientation
    //The linear and angular velocity are expressed with respect the current orientation frame. But if it's going ahead, it is going ahead with respect the 
    //rotated frame. So I must bring the velocity vector to the body frame
    Eigen::Vector3d imu_ang_vel_eig;
    imu_ang_vel_eig << imu_ang_vel.getX(), imu_ang_vel.getY(), imu_ang_vel.getZ();
    auto bodyAngularVel_ = rot_*imu_ang_vel_eig; //R^0_1*V^1
  
  }     


  inline Eigen::Matrix3d QuatToMat(Eigen::VectorXd Quat){
    Eigen::Matrix3d Rot;
    float s = Quat[0];
    float x = Quat[1];
    float y = Quat[2];
    float z = Quat[3];
    Rot << 1-2*(y*y+z*z),2*(x*y-s*z),2*(x*z+s*y),
    2*(x*y+s*z),1-2*(x*x+z*z),2*(y*z-s*x),
    2*(x*z-s*y),2*(y*z+s*x),1-2*(x*x+y*y);
    return Rot;
  }


  uint16_t convert_to_bits(double rad){
      double angle = rad * 180.0 / M_PI + 150.0;
      uint16_t bits = angle * 1024.0 / 300.0;
      if (bits < 0){
        bits = 0;
      }
      if (bits > 1023)  
      {
        bits = 1023;
      }
      
      return bits;
  }


private:
 
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
  uint8_t dxl_error_ = 0;

  //hebi
  bool ActuatorConnected_ = true;
  hebi::Lookup lookup_;
  std::shared_ptr<hebi::Group> group_;
  hebi::GroupFeedback Gfeedback_ = hebi::GroupFeedback(2);

  Eigen::VectorXd quat_;
  Eigen::Matrix3d rot_;

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Monopod>());
  rclcpp::shutdown();
  return 0;
}
