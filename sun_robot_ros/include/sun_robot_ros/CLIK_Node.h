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

typedef boost::function<TooN::Vector<>(void)> KF_JAC_FCN;

class Clik_Node {

private:

Clik_Node(); //No default constructor        

protected:

//The robot object
RobotPtr _robot;

//Clik vars
Vector<6,int> _mask;// <- mask, bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the error computation
double _clik_gain;// <- CLIK Gain
double _hz;// <- hz, frequency
double _second_obj_gain; // <- Gain for second objective  
Vector<> _joint_target; //<- target for joint position (used into the second objective obj)
Vector<> _joint_weights;

//ros
ros::NodeHandle _nh;

public:

/*CONSTRUCTORS*/

Clik_Node(const Robot& robot, const ros::NodeHandle& nh)
    :_robot( robot->clone() ),
    _nh(nh)
    {}

/* ROS CALLBKs */
Vector<3> pos_d;
Vector<3> dpos_d;
UnitQuaternion quat_d;
Vector<3> w_d;
void desiredPoseTwist_cbk( const sun_robot_msgs::PoseTwistStamped::ConstPtr& pose_twist_msg ){

    pos_d[0] = pose_twist_msg->pose.position.x;
    pos_d[1] = pose_twist_msg->pose.position.y;
    pos_d[2] = pose_twist_msg->pose.position.z;

    quat_d = UnitQuaternion( 
                makeVector(
                    pose_twist_msg->pose.orientation.x,
                    pose_twist_msg->pose.orientation.y,
                    pose_twist_msg->pose.orientation.z
                    ),
                pose_twist_msg->pose.orientation.w
                );
                
    dpos_d[0] = pose_twist_msg->twist.linear.x;
    dpos_d[1] = pose_twist_msg->twist.linear.y;
    dpos_d[2] = pose_twist_msg->twist.linear.z;

    w_d[0] = pose_twist_msg->twist.angular.x;
    w_d[1] = pose_twist_msg->twist.angular.y;
    w_d[2] = pose_twist_msg->twist.angular.z;

}


/* RUNNERS */

void run(){

    //Initialize subscribers
    ros::Subscriber desired_pose_twist_sub = nh.subscribe( _desired_pose_twist_topic_str, 1, desiredPoseTwist_cbk);

    //Wait for initial configuration
    Vector<> qDH_k = robot->joints_Robot2DH( getJointPosition_fcn() );

    //Initialize vars
    {
        Matrix<4,4> b_T_e = robot->fkine(qDH_k);
        pos_d = transl(b_T_e);
        quat_d = UnitQuaternion(b_T_e);
        dpos_d = Zeros;
        w_d = Zeros;
    }
    UnitQuaternion oldQ(quat_d);
    Vector<> dqDH = Zeros(robot->getNumJoints());
    Vector<6> error = Ones;

    ros::Rate loop_rate(_hz);

    while(ros::ok()){

        ros::spinOnce();

        qDH_k = //<- qDH at time k+1
            robot->clik(   
            qDH_k, //<- qDH now, time k
            pos_d, // <- desired position
            quat_d, // <- desired quaternion
            oldQ,// <- quaternion at last time (to ensure continuity)
            dpos_d, // <- desired translational velocity
            w_d, //<- desired angular velocity
            _mask,// <- mask, bitmask, if the i-th element is 0 then the i-th operative space coordinate will not be used in the error computation
            _clik_gain*_hz,// <- CLIK Gain
            1.0/_hz,// <- Ts, sampling time
            _second_obj_gain*_hz, // <- Gain for second objective  
            _joint_target, //<- target for joint position (used into the second objective obj)
            _joint_weights, // <- weights for joints in the second objective       
            //Return Vars
            dqDH, // <- joints velocity at time k+1
            error, //<- error vector at time k
            oldQ // <- Quaternion at time k (usefull for continuity in the next call of these function)
        );

        Vector<> qR = robot->joints_DH2Robot( qDH_k );
        Vector<> dqR = robot->jointsvel_DH2Robot( dqDH );

        //check limits
        if( robot->exceededHardJointLimits( qR ) ){
            cout << "ERROR ROBOT JOINT LIMITS!! On joints:" << endl;
            cout << robot->jointsNameFromBitMask( robot->checkHardJointLimits(qR) ) << endl;
            exit(-1);
        }
        if( robot->exceededHardVelocityLimits( dqR ) ){
            cout << "ERROR ROBOT Velocity!! On joints:" << endl;
            cout << robot->jointsNameFromBitMask( robot->checkHardVelocityLimits( dqR ) ) << endl;
            exit(-1);
        }

        publish_fcn( qR, dqR );

        loop_rate.sleep();

    }

}

}