/*

    Change the frame of a wrench

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

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "UnitQuaternion.h"

using namespace TooN;
using namespace std;

ros::Publisher pub_wrench_out;

Vector<3> pos;
UnitQuaternion uq;
bool b_only_rotation;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg){

    if(b_only_rotation){
        pos = Zeros;
    } else{
        pos[0] = pose_msg->pose.position.x;
        pos[1] = pose_msg->pose.position.y;
        pos[2] = pose_msg->pose.position.z;
    }

    uq = UnitQuaternion(
            pose_msg->pose.orientation.w,
            makeVector(
                pose_msg->pose.orientation.x,
                pose_msg->pose.orientation.y,
                pose_msg->pose.orientation.z
            )
    );

}

void wrench_in_cb(const geometry_msgs::WrenchStamped::ConstPtr& wrench_msg){

    Vector<3> force = makeVector(
                        wrench_msg->wrench.force.x,
                        wrench_msg->wrench.force.y,
                        wrench_msg->wrench.force.z
    );

    Vector<3> torque = makeVector(
                        wrench_msg->wrench.torque.x,
                        wrench_msg->wrench.torque.y,
                        wrench_msg->wrench.torque.z
    );

    torque = uq.mtimes( (pos ^ force) + torque );
    force = uq.mtimes(force);

    geometry_msgs::WrenchStamped out_msg;

    out_msg.header = wrench_msg->header;
    out_msg.wrench.force.x = force[0];
    out_msg.wrench.force.y = force[1];
    out_msg.wrench.force.z = force[2];
    out_msg.wrench.torque.x = torque[0];
    out_msg.wrench.torque.y = torque[1];
    out_msg.wrench.torque.z = torque[2];

    pub_wrench_out.publish(out_msg);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "wranch_ch_frame");

    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();
    
    string topic_wrench_in_str;
    nh_private.param("topic_wrench_in" , topic_wrench_in_str, string("wrench_in") );
    string topic_wrench_out_str;
    nh_private.param("topic_wrench_out" , topic_wrench_out_str, string("wrench_out") );
    string topic_pose_str;
    nh_private.param("topic_pose" , topic_pose_str, string("pose") );
    nh_private.param("use_rotation_only" , b_only_rotation, false );

    //Subs
    ros::Subscriber sub_wrench_in = nh_public.subscribe(topic_wrench_in_str, 1, wrench_in_cb);
    ros::Subscriber sub_pose = nh_public.subscribe(topic_pose_str, 1, pose_cb);

    //Pubs
    pub_wrench_out = nh_public.advertise<geometry_msgs::WrenchStamped>(topic_wrench_out_str, 1);
    
    ros::spin();

    return 0;
}