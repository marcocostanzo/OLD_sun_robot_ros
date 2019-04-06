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
            const string& desired_q_topic, 
            const string& set_mode_service,
            const string& get_status_service,
            const string& set_params_service,
            const CLIK_GET_QR_FCN& getJointPosition_fcn, 
            const CLIK_PUBLISH_QR_FCN& publish_fcn,
            double clik_gain,
            double hz,
            double second_obj_gain,
            const Vector<>& joint_target_robot,
            const Vector<>& joint_weights,
            const Vector<6,int>& mask_cartesian
            )
    :_robot( robot.clone() ),
    _nh(nh),
    _desired_pose_twist_topic_str(desired_pose_twist_topic),
    _desired_q_topic_str(desired_q_topic),
    _service_set_mode_str(set_mode_service),
    _service_get_status_str(get_status_service),
    _service_set_params_str(set_params_service),
    _getJointPosition_fcn(getJointPosition_fcn),
    _publish_fcn(publish_fcn),
    _clik_gain(clik_gain),
    _hz(hz),
    _second_obj_gain(second_obj_gain),
    _joint_target_dh( robot.joints_Robot2DH(joint_target_robot) ),
    _joint_weights(joint_weights),
    _mask_cartesian(mask_cartesian),
    _qDH_k( Zeros( robot.getNumJoints() ) ),
    _dqDH( Zeros( robot.getNumJoints() ) ),
    _mode(sun_robot_msgs::ClikSetMode::Request::MODE_STOP),
    _loop_rate(hz)
    {}

CLIK_Node::CLIK_Node(  
            const Robot& robot, 
            const ros::NodeHandle& nh_for_topics,
            const ros::NodeHandle& nh_for_parmas,
            const CLIK_GET_QR_FCN& getJointPosition_fcn, 
            const CLIK_PUBLISH_QR_FCN& publish_fcn
            )
    :_robot( robot.clone() ),
    _nh(nh_for_topics),
    _getJointPosition_fcn(getJointPosition_fcn),
    _publish_fcn(publish_fcn),
    _joint_target_dh(Zeros(robot.getNumJoints())),
    _joint_weights(Zeros(robot.getNumJoints())),
    _mask_cartesian(Ones),
    _qDH_k( Zeros( robot.getNumJoints() ) ),
    _dqDH( Zeros( robot.getNumJoints() ) ),
    _mode(sun_robot_msgs::ClikSetMode::Request::MODE_STOP),
    _loop_rate(1000.0)
{
    //params
    nh_for_parmas.param("desired_pose_twist_topic" , _desired_pose_twist_topic_str, string("desired_pose_twist") );
    nh_for_parmas.param("desired_q_topic" , _desired_q_topic_str, string("desired_q") );
    nh_for_parmas.param("set_mode_service" , _service_set_mode_str, string("set_mode") );
    nh_for_parmas.param("get_status_service" , _service_get_status_str, string("get_status") );
    nh_for_parmas.param("set_params_service" , _service_set_params_str, string("set_params") );
    nh_for_parmas.param("clik_gain" , _clik_gain, 0.5 );
    nh_for_parmas.param("hz" , _hz, 1000.0 );
    _loop_rate = ros::Rate(_hz);
    double dls_joint_speed_saturation;
    nh_for_parmas.param("dls_joint_speed_saturation" , dls_joint_speed_saturation, 5.0 );
    _robot->setDLSJointSpeedSaturation(dls_joint_speed_saturation);
    nh_for_parmas.param("second_obj_gain" , _second_obj_gain, 0.0 );

    vector<double> joint_target_robot_std;
    nh_for_parmas.getParam("joint_target_robot", joint_target_robot_std);
    if(joint_target_robot_std.size() != _robot->getNumJoints()){
        cout << HEADER_PRINT BOLDRED "Error: joint_target_robot size mismatch" CRESET << endl;
        exit(-1);
    }
    _joint_target_dh = _robot->joints_Robot2DH( wrapVector(joint_target_robot_std.data(), joint_target_robot_std.size()) );

    vector<double> joint_weights_std;
    nh_for_parmas.getParam("joint_weights", joint_weights_std);
    if(joint_weights_std.size() != _robot->getNumJoints()){
        cout << HEADER_PRINT BOLDRED "Error: joint_weights size mismatch" CRESET << endl;
        exit(-1);
    }
    _joint_weights = wrapVector(joint_weights_std.data(), joint_weights_std.size());

    vector<int> mask_cartesian_std;
    nh_for_parmas.getParam("mask_cartesian", mask_cartesian_std);
    if(mask_cartesian_std.size() != 6){
        cout << HEADER_PRINT BOLDRED "Error: mask_cartesian size mismatch" CRESET << endl;
        exit(-1);
    }
    _mask_cartesian = wrapVector(mask_cartesian_std.data(), mask_cartesian_std.size());

    vector<double> n_T_e_position_std;
    nh_for_parmas.getParam("n_T_e_position", n_T_e_position_std);
    if(n_T_e_position_std.size() != 3){
        cout << HEADER_PRINT BOLDRED "Error: n_T_e_position size mismatch" CRESET << endl;
        exit(-1);
    }
    Vector<3> n_T_e_position = wrapVector(n_T_e_position_std.data(), n_T_e_position_std.size());

    vector<double> n_T_e_quaternion_std;
    nh_for_parmas.getParam("n_T_e_quaternion", n_T_e_quaternion_std);
    if(n_T_e_quaternion_std.size() != 4){
        cout << HEADER_PRINT BOLDRED "Error: n_T_e_quaternion size mismatch" CRESET << endl;
        exit(-1);
    }
    UnitQuaternion n_T_e_quaternion( wrapVector(n_T_e_quaternion_std.data(), n_T_e_quaternion_std.size()) );

    Matrix<4,4> n_T_e = transl(n_T_e_position);
    n_T_e.slice<0,0,3,3>() = n_T_e_quaternion.torot();

    _robot->setnTe(n_T_e);

}

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

void CLIK_Node::desiredQ_cbk( const sun_robot_msgs::JointPositionVelocityStamped::ConstPtr& joi_pos_vel_msg ){

    if(_mode == sun_robot_msgs::ClikSetMode::Request::MODE_JOINT){

        Vector<> qR = wrapVector(joi_pos_vel_msg->position.data(), joi_pos_vel_msg->position.size());
        Vector<> dqR = wrapVector(joi_pos_vel_msg->velocity.data(), joi_pos_vel_msg->velocity.size());

        if( (qR.size() != _robot->getNumJoints()) || (dqR.size() != _robot->getNumJoints()) ){
            cout << HEADER_PRINT RED "ERROR: MODE_JOINT size mismatch" CRESET << endl;
            return;
        }

        safety_check(qR,dqR);

        _publish_fcn(qR,dqR);

    }

}

bool CLIK_Node::setMode(sun_robot_msgs::ClikSetMode::Request  &req, 
   		 		sun_robot_msgs::ClikSetMode::Response &res){

    switch (req.mode)
    {
        case sun_robot_msgs::ClikSetMode::Request::MODE_STOP:{
            cout << HEADER_PRINT YELLOW "Stopped!" << CRESET << endl;
            break;
        }
        case sun_robot_msgs::ClikSetMode::Request::MODE_CLIK:{
            cout << HEADER_PRINT YELLOW "CLIK MODE" << CRESET << endl;
            break;
        }
        case sun_robot_msgs::ClikSetMode::Request::MODE_JOINT:{
            cout << HEADER_PRINT YELLOW "JOINT MODE" << CRESET << endl;
            break;
        }
        default:{
            cout << HEADER_PRINT RED "Error in setMode(): Invalid mode in request!" CRESET << endl;
            res.success = false;
            return true;
        }
    }

    if( _mode != req.mode ){
        refresh();
    }
    _mode = req.mode;
    
    res.success = true;
	return true;	
}

bool CLIK_Node::getStatus(sun_robot_msgs::ClikStatus::Request  &req, 
   		 		sun_robot_msgs::ClikStatus::Response &res){

    if(req.refresh){
        refresh();
    }

    //res.mode = _mode;

    res.qR.resize(_robot->getNumJoints());
    Vector<> qR = _robot->joints_DH2Robot(_qDH_k);
    res.qR[0] = qR[0];
    res.qR[1] = qR[1];
    res.qR[2] = qR[2];
    res.qR[3] = qR[3];
    res.qR[4] = qR[4];
    res.qR[5] = qR[5];
    res.qR[6] = qR[6];

    Matrix<4,4> b_T_e = _robot->fkine(_qDH_k);
    Vector<3> pos = transl(b_T_e);
    UnitQuaternion quat = UnitQuaternion(b_T_e);

    res.pose.position.x = pos[0];
    res.pose.position.y = pos[1];
    res.pose.position.z = pos[2];

    res.pose.orientation.w = quat.getS();
    Vector<3> quat_v = quat.getV();
    res.pose.orientation.x = quat_v[0];
    res.pose.orientation.y = quat_v[1];
    res.pose.orientation.z = quat_v[2];

    res.success = true;
    return true;

}

bool CLIK_Node::setParams(sun_robot_msgs::ClikSetParams::Request  &req, 
   		 		sun_robot_msgs::ClikSetParams::Response &res){

    if( ( req.joint_target_robot.size() != _robot->getNumJoints() ) || ( req.joint_weights.size() != _robot->getNumJoints() ) ){
        
        cout << HEADER_PRINT RED "Error in setParams(): joint size mismatch" CRESET << endl;

        res.success = false;
        return true;
    }

    _clik_gain = req.gain;
    _hz = req.hz;
    _loop_rate = ros::Rate(_hz);
    _robot->setDLSJointSpeedSaturation( req.dls_joint_speed_saturation );
    _second_obj_gain = req.gain_2_obj;
    _joint_target_dh = _robot->joints_Robot2DH( 
                        wrapVector(
                            req.joint_target_robot.data(), 
                            req.joint_target_robot.size()
                        ) 
                        );
    _joint_weights = wrapVector(
                        req.joint_weights.data(), 
                        req.joint_weights.size()
                    ); 

    res.success = true;
    return true;

}

/* END ROS CBs */

/* RUNNERS */
void CLIK_Node::refresh(){
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
    ros::Subscriber desired_q_sub = _nh.subscribe( _desired_q_topic_str, 1, &CLIK_Node::desiredQ_cbk, this);

    //Init Services
    ros::ServiceServer serviceSetMode = _nh.advertiseService( _service_set_mode_str, &CLIK_Node::setMode, this);
    ros::ServiceServer serviceGetClikStatus = _nh.advertiseService( _service_get_status_str, &CLIK_Node::getStatus, this);
    ros::ServiceServer serviceSetClikParams = _nh.advertiseService( _service_set_params_str, &CLIK_Node::setParams, this);

    refresh();

    while(ros::ok()){

        ros::spinOnce();
        
        switch (_mode)
        {
            case sun_robot_msgs::ClikSetMode::Request::MODE_STOP:{
                break;
            }
            case sun_robot_msgs::ClikSetMode::Request::MODE_CLIK:{
                clik_loop();
                break;
            }
            case sun_robot_msgs::ClikSetMode::Request::MODE_JOINT:{
                break;
            }
            default:{
                cout << HEADER_PRINT BOLDRED "Error in main while: Invalid mode!" CRESET << endl;
                exit(-1);
            }
        }

        _loop_rate.sleep();

    }

}

void CLIK_Node::clik_loop(){

    _qDH_k = //<- qDH at time k+1
        _robot->clik(   
        _qDH_k, //<- qDH now, time k
        _pos_d, // <- desired position
        _quat_d, // <- desired quaternion
        _oldQ,// <- quaternion at last time (to ensure continuity)
        _dpos_d, // <- desired translational velocity
        _w_d, //<- desired angular velocity
        _mask_cartesian,// <- mask, bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the error computation
        _clik_gain*_hz,// <- CLIK Gain
        1.0/_hz,// <- Ts, sampling time
        _second_obj_gain*_hz, // <- Gain for second objective  
        _joint_target_dh, //<- target for joint position (used into the second objective obj)
        _joint_weights, // <- weights for joints in the second objective       
        //Return Vars
        _dqDH, // <- joints velocity at time k+1
        _error, //<- error vector at time k
        _oldQ // <- Quaternion at time k (usefull for continuity in the next call of these function)
    );

    Vector<> qR = _robot->joints_DH2Robot( _qDH_k );
    Vector<> dqR = _robot->jointsvel_DH2Robot( _dqDH );

    safety_check(qR, dqR);

    _publish_fcn( qR, dqR );

}

void CLIK_Node::safety_check(const Vector<>& qR, const Vector<>& dqR){

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

}