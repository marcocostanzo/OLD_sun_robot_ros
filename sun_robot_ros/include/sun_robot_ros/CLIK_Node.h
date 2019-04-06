/*

    Clik Robot Class

    Copyright 2019 Università della Campania Luigi Vanvitelli

    Author: Marco Costanzo <marco.costanzo@unicampania.it>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef CLIK_NODE_H
#define CLIK_NODE_H

#include "Robot.h"
#include "ros/ros.h"
#include "sun_robot_msgs/PoseTwistStamped.h"
#include "sun_robot_msgs/JointPositionVelocityStamped.h"
#include "sun_robot_msgs/ClikStatus.h"
#include "sun_robot_msgs/ClikSetMode.h"
#include "sun_robot_msgs/ClikSetParams.h"

//Function handles definition
typedef boost::function<TooN::Vector<>(void)> CLIK_GET_QR_FCN;
typedef boost::function<void(const TooN::Vector<>&,const TooN::Vector<>&)> CLIK_PUBLISH_QR_FCN;

class CLIK_Node {

private:

CLIK_Node(); //No default constructor        

protected:

//The robot object
RobotPtr _robot;

//Clik vars
TooN::Vector<6,int> _mask_cartesian;// <- mask, bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the error computation
double _clik_gain;// <- CLIK Gain
double _hz;// <- hz, frequency
double _second_obj_gain; // <- Gain for second objective  
TooN::Vector<> _joint_target_dh; //<- target for joint position (used into the second objective obj)
TooN::Vector<> _joint_weights;

//external functions
CLIK_GET_QR_FCN _getJointPosition_fcn;
CLIK_PUBLISH_QR_FCN _publish_fcn;

//ros
ros::NodeHandle _nh;
ros::Rate _loop_rate;
std::string _desired_pose_twist_topic_str;
std::string _desired_q_topic_str;
std::string _service_set_mode_str;
std::string _service_get_status_str;
std::string _service_set_params_str;

//mode
uint8_t _mode; // Stop/CLIK/Joint


public:

/*CONSTRUCTORS*/
CLIK_Node(  
            const Robot& robot, 
            const ros::NodeHandle& nh,
            const std::string& desired_pose_twist_topic, 
            const std::string& desired_q_topic, 
            const std::string& set_mode_service,
            const std::string& get_status_service,
            const std::string& set_params_service,
            const CLIK_GET_QR_FCN& getJointPosition_fcn, 
            const CLIK_PUBLISH_QR_FCN& publish_fcn,
            double clik_gain,
            double hz,
            double second_obj_gain,
            const TooN::Vector<>& joint_target_dh,
            const TooN::Vector<>& joint_weights,
            const TooN::Vector<6,int>& mask_cartesian = TooN::Ones
            );

CLIK_Node(  
            const Robot& robot, 
            const ros::NodeHandle& nh_for_topics,
            const ros::NodeHandle& nh_for_parmas,
            const CLIK_GET_QR_FCN& getJointPosition_fcn, 
            const CLIK_PUBLISH_QR_FCN& publish_fcn
            );

protected:
/* ROS CALLBKs */
TooN::Vector<3> _pos_d;
TooN::Vector<3> _dpos_d;
UnitQuaternion _quat_d;
TooN::Vector<3> _w_d;
void desiredPoseTwist_cbk( const sun_robot_msgs::PoseTwistStamped::ConstPtr& pose_twist_msg );

void desiredQ_cbk( const sun_robot_msgs::JointPositionVelocityStamped::ConstPtr& joi_pos_vel_msg );

bool setMode(sun_robot_msgs::ClikSetMode::Request  &req, 
   		 		sun_robot_msgs::ClikSetMode::Response &res);

bool getStatus(sun_robot_msgs::ClikStatus::Request  &req, 
   		 		sun_robot_msgs::ClikStatus::Response &res);

bool setParams(sun_robot_msgs::ClikSetParams::Request  &req, 
   		 		sun_robot_msgs::ClikSetParams::Response &res);

/* END ROS CBs */


/* RUNNERS */
TooN::Vector<> _qDH_k;
UnitQuaternion _oldQ;
TooN::Vector<> _dqDH;
TooN::Vector<6> _error;

public:
void refresh();

void run();

protected:
void clik_loop();

void safety_check(const TooN::Vector<>& qR, const TooN::Vector<>& dqR);

}; //END CLASS

#endif