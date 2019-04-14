/*

    Precompiler definitions for ROBOT_AS class

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

#ifndef SUN_ROBOT_AS_PRECOMPILER_H
#define SUN_ROBOT_AS_PRECOMPILER_H

/* PRECOMPILER FCNS FOR JOINT ACTIONS */

#define _ROBOT_AS_GENERATE_JOINT_ABORT_HEADER( fcn_name) \
void fcn_name( const std::string& msg );

#define _ROBOT_AS_GENERATE_JOINT_ABORT( fcn_name, action_name, action_server) \
void Robot_AS::fcn_name( const string& msg ) { \
    sun_robot_msgs::action_name##Result result; \
    result.success = false; \
    result.msg = msg; \
    action_server.setAborted(result); \
}

#define _ROBOT_AS_GENERATE_JOINT_PUBLISH_FEEDBACK_HEADER( fcn_name ) \
void fcn_name( const TooN::Vector<>& qR, const TooN::Vector<>& dqR );

#define _ROBOT_AS_GENERATE_JOINT_PUBLISH_FEEDBACK( fcn_name, feedbk_msg, action_server ) \
void Robot_AS::fcn_name( const Vector<>& qR, const Vector<>& dqR ){ \
    for(int i=0; i<_num_joints; i++){ \
        feedbk_msg .positions[i] = qR[i]; \
        feedbk_msg .velocities[i] = dqR[i]; \
    } \
    action_server.publishFeedback( feedbk_msg ); \
}

#define _ROBOT_AS_GENERATE_JOINT_IS_PREEPMTED_REQUESTED_HEADER( fcn_name ) \
bool fcn_name();

#define _ROBOT_AS_GENERATE_JOINT_IS_PREEPMTED_REQUESTED( fcn_name, action_server ) \
bool Robot_AS::fcn_name(){ \
    return action_server.isPreemptRequested(); \
}

#define _ROBOT_AS_GENERATE_JOINT_PREEMPTED_HEADER( fcn_name ) \
void fcn_name();

#define _ROBOT_AS_GENERATE_JOINT_PREEMPTED( fcn_name, action_name, action_server ) \
void Robot_AS::fcn_name(){ \
    sun_robot_msgs::action_name##Result result; \
    result.success = true; \
    result.msg = "Preempted"; \
    action_server.setPreempted( result ); \
}

#define _ROBOT_AS_GENERATE_JOINT_SUCCESS_HEADER( fcn_name ) \
void fcn_name();

#define _ROBOT_AS_GENERATE_JOINT_SUCCESS( fcn_name, action_name, action_server ) \
void Robot_AS::fcn_name(){ \
    sun_robot_msgs::action_name##Result result; \
    result.success = true; \
    result.msg = "Compleate"; \
    action_server.setSucceeded( result ); \
}

#define _ROBOT_AS_GENERATE_JOINT_FCNS_HEADER_ALL( fcn_prefix ) \
_ROBOT_AS_GENERATE_JOINT_ABORT_HEADER( fcn_prefix##Abort ) \
_ROBOT_AS_GENERATE_JOINT_PUBLISH_FEEDBACK_HEADER( fcn_prefix##PublishFeedback ) \
_ROBOT_AS_GENERATE_JOINT_IS_PREEPMTED_REQUESTED_HEADER( fcn_prefix##IsPreemptRequested ) \
_ROBOT_AS_GENERATE_JOINT_PREEMPTED_HEADER( fcn_prefix##Preempted ) \
_ROBOT_AS_GENERATE_JOINT_SUCCESS_HEADER( fcn_prefix##Success ) 

#define _ROBOT_AS_GENERATE_JOINT_FCNS_ALL( fcn_prefix, action_name, feedbk_msg, action_server ) \
_ROBOT_AS_GENERATE_JOINT_ABORT( fcn_prefix##Abort , action_name, action_server) \
_ROBOT_AS_GENERATE_JOINT_PUBLISH_FEEDBACK( fcn_prefix##PublishFeedback , feedbk_msg, action_server ) \
_ROBOT_AS_GENERATE_JOINT_IS_PREEPMTED_REQUESTED( fcn_prefix##IsPreemptRequested , action_server ) \
_ROBOT_AS_GENERATE_JOINT_PREEMPTED( fcn_prefix##Preempted , action_name, action_server ) \
_ROBOT_AS_GENERATE_JOINT_SUCCESS( fcn_prefix##Success , action_name, action_server ) 

/* end PRECOMPILER FCNS FOR JOINT ACTIONS */

/* PRECOMPILER FCNS FOR CARTESIAN ACTIONS */

#define _ROBOT_AS_GENERATE_CARTESIAN_ABORT_HEADER( fcn_name ) \
_ROBOT_AS_GENERATE_JOINT_ABORT_HEADER( fcn_name )

#define _ROBOT_AS_GENERATE_CARTESIAN_ABORT( fcn_name, action_name, action_server) \
_ROBOT_AS_GENERATE_JOINT_ABORT( fcn_name , action_name, action_server)

#define _ROBOT_AS_GENERATE_CARTESIAN_PUBLISH_FEEDBACK_HEADER( fcn_name ) \
void fcn_name( \
                double time_left, \
                const TooN::Vector<3>& pos, \
                const UnitQuaternion& quat, \
                const TooN::Vector<3>& vel, \
                const TooN::Vector<3>& w );

#define _ROBOT_AS_GENERATE_CARTESIAN_PUBLISH_FEEDBACK( fcn_name, feedbk_msg, action_server ) \
void Robot_AS::fcn_name( \
                        double time_left, \
                        const Vector<3>& pos, \
                        const UnitQuaternion& quat, \
                        const Vector<3>& vel, \
                        const Vector<3>& w){ \
 \
    feedbk_msg.time_left = time_left; \
 \
    feedbk_msg.pose.position.x = pos[0]; \
    feedbk_msg.pose.position.y = pos[1]; \
    feedbk_msg.pose.position.z = pos[2]; \
 \
    feedbk_msg.pose.orientation.w = quat.getS(); \
    Vector<3> quat_v = quat.getV(); \
    feedbk_msg.pose.orientation.x = quat_v[0]; \
    feedbk_msg.pose.orientation.y = quat_v[1]; \
    feedbk_msg.pose.orientation.z = quat_v[2]; \
 \
    feedbk_msg.twist.linear.x = vel[0]; \
    feedbk_msg.twist.linear.y = vel[1]; \
    feedbk_msg.twist.linear.z = vel[2]; \
 \
    feedbk_msg.twist.angular.x = w[0]; \
    feedbk_msg.twist.angular.y = w[1]; \
    feedbk_msg.twist.angular.z = w[2]; \
 \
    action_server.publishFeedback(feedbk_msg); \
}

#define _ROBOT_AS_GENERATE_CARTESIAN_IS_PREEPMTED_REQUESTED_HEADER( fcn_name ) \
_ROBOT_AS_GENERATE_JOINT_IS_PREEPMTED_REQUESTED_HEADER( fcn_name )

#define _ROBOT_AS_GENERATE_CARTESIAN_IS_PREEPMTED_REQUESTED( fcn_name, action_server ) \
_ROBOT_AS_GENERATE_JOINT_IS_PREEPMTED_REQUESTED( fcn_name, action_server )

#define _ROBOT_AS_GENERATE_CARTESIAN_PREEMPTED_HEADER( fcn_name ) \
_ROBOT_AS_GENERATE_JOINT_PREEMPTED_HEADER( fcn_name )

#define _ROBOT_AS_GENERATE_CARTESIAN_PREEMPTED( fcn_name, action_name, action_server ) \
_ROBOT_AS_GENERATE_JOINT_PREEMPTED( fcn_name, action_name, action_server )

#define _ROBOT_AS_GENERATE_CARTESIAN_SUCCESS_HEADER( fcn_name ) \
_ROBOT_AS_GENERATE_JOINT_SUCCESS_HEADER( fcn_name )

#define _ROBOT_AS_GENERATE_CARTESIAN_SUCCESS( fcn_name, action_name, action_server ) \
_ROBOT_AS_GENERATE_JOINT_SUCCESS( fcn_name, action_name, action_server )

#define _ROBOT_AS_GENERATE_CARTESIAN_FCNS_HEADER_ALL( fcn_prefix ) \
_ROBOT_AS_GENERATE_CARTESIAN_ABORT_HEADER( fcn_prefix##Abort ) \
_ROBOT_AS_GENERATE_CARTESIAN_PUBLISH_FEEDBACK_HEADER( fcn_prefix##PublishFeedback ) \
_ROBOT_AS_GENERATE_CARTESIAN_IS_PREEPMTED_REQUESTED_HEADER( fcn_prefix##IsPreemptRequested ) \
_ROBOT_AS_GENERATE_CARTESIAN_PREEMPTED_HEADER( fcn_prefix##Preempted ) \
_ROBOT_AS_GENERATE_CARTESIAN_SUCCESS_HEADER( fcn_prefix##Success ) 

#define _ROBOT_AS_GENERATE_CARTESIAN_FCNS_ALL( fcn_prefix, action_name, feedbk_msg, action_server ) \
_ROBOT_AS_GENERATE_CARTESIAN_ABORT( fcn_prefix##Abort , action_name, action_server) \
_ROBOT_AS_GENERATE_CARTESIAN_PUBLISH_FEEDBACK( fcn_prefix##PublishFeedback , feedbk_msg, action_server ) \
_ROBOT_AS_GENERATE_CARTESIAN_IS_PREEPMTED_REQUESTED( fcn_prefix##IsPreemptRequested , action_server ) \
_ROBOT_AS_GENERATE_CARTESIAN_PREEMPTED( fcn_prefix##Preempted , action_name, action_server ) \
_ROBOT_AS_GENERATE_CARTESIAN_SUCCESS( fcn_prefix##Success , action_name, action_server ) 

/* end PRECOMPILER FCNS FOR CARTESIAN ACTIONS */

#endif