/*
    ROS node to filter wrench stamped

    Copyright 2018 Università della Campania Luigi Vanvitelli

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

#include "geometry_msgs/WrenchStamped.h"
#include "TF_MIMO/TF_MIMO_DIAGONAL.h"
#include "TF_SISO/TF_FIRST_ORDER_FILTER.h"


using namespace TooN;
using namespace std;

ros::Publisher pubWrenchFilter;
geometry_msgs::WrenchStamped msgWrenchFilter;

Vector<6> wrench;

//==========TOPICs CALLBKs=========//
void readV( const geometry_msgs::WrenchStamped::ConstPtr& msg  ){

    wrench[0] = msg->wrench.force.x;
    wrench[1] = msg->wrench.force.y;
    wrench[2] = msg->wrench.force.z;
    wrench[3] = msg->wrench.torque.x;
    wrench[4] = msg->wrench.torque.y;
    wrench[5] = msg->wrench.torque.z;

	msgWrenchFilter.header.stamp = msg->header.stamp;
    msgWrenchFilter.header.frame_id = msg->header.frame_id;
	
}

//====================================//


int main(int argc, char *argv[])
{

    ros::init(argc,argv,"filter_wrench_stamped");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh_public = ros::NodeHandle();

    /**** PARAMS ****/
    string str_in_topic = string("");
    nh_private.param("in_topic" , str_in_topic, string("/tactile") );
    string str_out_topic = string("");
    nh_private.param("out_topic" , str_out_topic, str_in_topic + string("/filter") );
    double cut_freq;
    nh_private.param("cut_freq" , cut_freq, 20.0 );
    double Hz;
    nh_private.param("rate" , Hz, 500.0 );
	/************************************/

    /******INIT ROS MSGS**********/
	/********************/

    /*******INIT ROS PUB**********/
	//Wrench_filter pub
	pubWrenchFilter = nh_public.advertise<geometry_msgs::WrenchStamped>( str_out_topic, 1);
    /***************************/

    /*******INIT ROS SUB**********/
	ros::Subscriber subWrench = nh_public.subscribe( str_in_topic , 1, readV);
    /***************************/

    /******INIT FILTER************/
    TF_MIMO_DIAGONAL filter(    6,
                                TF_FIRST_ORDER_FILTER(cut_freq, 1.0/Hz), 
                                1.0/Hz
                            );
    /***************************/	

    /*============LOOP==============*/
    ros::Rate loop_rate(Hz);
	while(ros::ok()){

        Vector<6> wrench_filter = filter.apply( wrench );
	
        //Fill msg
		msgWrenchFilter.wrench.force.x = wrench_filter[0];
        msgWrenchFilter.wrench.force.y = wrench_filter[1];
        msgWrenchFilter.wrench.force.z = wrench_filter[2];
        msgWrenchFilter.wrench.torque.x = wrench_filter[3];
        msgWrenchFilter.wrench.torque.y = wrench_filter[4];
        msgWrenchFilter.wrench.torque.z = wrench_filter[5];

		pubWrenchFilter.publish( msgWrenchFilter );
		loop_rate.sleep();
      	ros::spinOnce();
			
	}
    
    /*==============================*/

    return 0;
}