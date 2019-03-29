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

#include "sun_robot_ros/CLIK_Node.h"

#ifndef SUN_COLORS
#define SUN_COLORS

/* ======= COLORS ========= */
#define CRESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLD    "\033[1m"       /* Bold */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */
/*===============================*/

#endif

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "][CLIK] " CRESET 

using namespace std;
using namespace TooN;

/*CONSTRUCTORS*/

CLIK_Node::CLIK_Node(  
            const Robot& robot, 
            const ros::NodeHandle& nh,
            const string& desired_pose_twist_topic, 
            const string& set_stopped_service,
            const CLIK_GET_QR_FCN& getJointPosition_fcn, 
            const CLIK_PUBLISH_QR_FCN& publish_fcn,
            double clik_gain,
            double second_obj_gain,
            const Vector<>& joint_target,
            const Vector<>& joint_weights,
            const Vector<6,int>& mask,
            bool start_stopped )
    :_robot( robot.clone() ),
    _nh(nh),
    _desired_pose_twist_topic_str(desired_pose_twist_topic),
    _service_set_stopped_str(set_stopped_service),
    _getJointPosition_fcn(getJointPosition_fcn),
    _publish_fcn(publish_fcn),
    _clik_gain(clik_gain),
    _second_obj_gain(second_obj_gain),
    _joint_target(joint_target),
    _joint_weights(joint_weights),
    _mask(mask),
    _stopped(start_stopped),
    _qDH_k( Zeros( robot.getNumJoints() ) ),
    _dqDH( Zeros( robot.getNumJoints() ) )
    {}


void CLIK_Node::desiredPoseTwist_cbk( const sun_robot_msgs::PoseTwistStamped::ConstPtr& pose_twist_msg ){

    _pos_d[0] = pose_twist_msg->pose.position.x;
    _pos_d[1] = pose_twist_msg->pose.position.y;
    _pos_d[2] = pose_twist_msg->pose.position.z;

    _quat_d = UnitQuaternion(
                pose_twist_msg->pose.orientation.w, 
                makeVector(
                    pose_twist_msg->pose.orientation.x,
                    pose_twist_msg->pose.orientation.y,
                    pose_twist_msg->pose.orientation.z
                    )
                );
                
    _dpos_d[0] = pose_twist_msg->twist.linear.x;
    _dpos_d[1] = pose_twist_msg->twist.linear.y;
    _dpos_d[2] = pose_twist_msg->twist.linear.z;

    _w_d[0] = pose_twist_msg->twist.angular.x;
    _w_d[1] = pose_twist_msg->twist.angular.y;
    _w_d[2] = pose_twist_msg->twist.angular.z;

}

bool CLIK_Node::setStopped(std_srvs::SetBool::Request  &req, 
   		 		std_srvs::SetBool::Response &res){
	_stopped = req.data;
    if(!_stopped){
        initClikVars();
    } else{
        cout << HEADER_PRINT YELLOW "Stopped!" << CRESET << endl;
    }
    res.success = true;
	return true;	
}

/* END ROS CBs */


/* RUNNERS */
void CLIK_Node::initClikVars(){
    //Wait for initial configuration
    cout << HEADER_PRINT YELLOW "Wait for joint positions..." << CRESET << endl;
    _qDH_k = _robot->joints_Robot2DH( _getJointPosition_fcn() );
    cout << HEADER_PRINT "Joint positions: " << _qDH_k << endl;
    //Initialize vars
    Matrix<4,4> b_T_e = _robot->fkine(_qDH_k);
    _pos_d = transl(b_T_e);
    _quat_d = UnitQuaternion(b_T_e);
    _dpos_d = Zeros;
    _w_d = Zeros;
    _oldQ = UnitQuaternion(_quat_d);
    _dqDH = Zeros(_robot->getNumJoints());
    _error = Ones;
    cout << HEADER_PRINT GREEN "Initialized!" << CRESET << endl;
}

void CLIK_Node::run(){

    //Initialize subscribers
    ros::Subscriber desired_pose_twist_sub = _nh.subscribe( _desired_pose_twist_topic_str, 1, &CLIK_Node::desiredPoseTwist_cbk, this);

    //Init Services
    ros::ServiceServer serviceSetStopped = _nh.advertiseService( _service_set_stopped_str, &CLIK_Node::setStopped, this);

    initClikVars();

    ros::Rate loop_rate(_hz);

    while(ros::ok()){

        ros::spinOnce();

        if(_stopped){
            loop_rate.sleep();
            continue;
        }

        _qDH_k = //<- qDH at time k+1
            _robot->clik(   
            _qDH_k, //<- qDH now, time k
            _pos_d, // <- desired position
            _quat_d, // <- desired quaternion
            _oldQ,// <- quaternion at last time (to ensure continuity)
            _dpos_d, // <- desired translational velocity
            _w_d, //<- desired angular velocity
            _mask,// <- mask, bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the error computation
            _clik_gain*_hz,// <- CLIK Gain
            1.0/_hz,// <- Ts, sampling time
            _second_obj_gain*_hz, // <- Gain for second objective  
            _joint_target, //<- target for joint position (used into the second objective obj)
            _joint_weights, // <- weights for joints in the second objective       
            //Return Vars
            _dqDH, // <- joints velocity at time k+1
            _error, //<- error vector at time k
            _oldQ // <- Quaternion at time k (usefull for continuity in the next call of these function)
        );

        Vector<> qR = _robot->joints_DH2Robot( _qDH_k );
        Vector<> dqR = _robot->jointsvel_DH2Robot( _dqDH );

        //check limits
        if( _robot->exceededHardJointLimits( qR ) ){
            cout << HEADER_PRINT BOLDRED "ERROR ROBOT JOINT LIMITS!! On joints:" <<
                    _robot->jointsNameFromBitMask( _robot->checkHardJointLimits(qR) ) << CRESET << endl;
            exit(-1);
        }
        if( _robot->exceededHardVelocityLimits( dqR ) ){
            cout << HEADER_PRINT BOLDRED "ERROR ROBOT Velocity!! On joints:" <<
                    _robot->jointsNameFromBitMask( _robot->checkHardVelocityLimits( dqR ) ) << CRESET << endl;
            exit(-1);
        }

        _publish_fcn( qR, dqR );

        loop_rate.sleep();

    }

}
