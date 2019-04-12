/*

    node that generates a sine wave

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
#include "std_msgs/Float64.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "Traj_Generators/Sine_Traj.h"

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

#define HEADER_PRINT BOLDYELLOW "[" << ros::this_node::getName() << "] " CRESET 

using namespace std;

constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

int msg_index;
ros::Publisher pub_out;

void pubFloat64( double data ){

    std_msgs::Float64 out_msg;
    out_msg.data = data;

    pub_out.publish( out_msg );

}

void pubWrench( double data ){

    geometry_msgs::Wrench out_msg;
    
    switch (msg_index)
    {
        case 0:{
            out_msg.force.x = data;
            break;
        }
        case 1:{
            out_msg.force.y = data;
            break;
        }
        case 2:{
            out_msg.force.z = data;
            break;
        }
        case 3:{
            out_msg.torque.x = data;
            break;
        }
        case 4:{
            out_msg.torque.y = data;
            break;
        }
        case 5:{
            out_msg.torque.z = data;
            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "ERROR: invalid msg_index(" << msg_index <<") for wrench " CRESET << endl;
            exit(-1);
        }
    }
    

    pub_out.publish( out_msg );

}

void pubWrenchStamped( double data ){

    geometry_msgs::WrenchStamped out_msg;

    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = "sine_wave";
    
    switch (msg_index)
    {
        case 0:{
            out_msg.wrench.force.x = data;
            break;
        }
        case 1:{
            out_msg.wrench.force.y = data;
            break;
        }
        case 2:{
            out_msg.wrench.force.z = data;
            break;
        }
        case 3:{
            out_msg.wrench.torque.x = data;
            break;
        }
        case 4:{
            out_msg.wrench.torque.y = data;
            break;
        }
        case 5:{
            out_msg.wrench.torque.z = data;
            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "ERROR: invalid msg_index(" << msg_index <<") for wrench stamped " CRESET << endl;
            exit(-1);
        }
    }
    

    pub_out.publish( out_msg );

}


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "sine_wave_generator");
    
    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();
    
    //params
    double hz;
    nh_private.param("hz" , hz, 300.0 );
    double duration;
    nh_private.param("duration" , duration, 0.0 );
    double amplitude;
    nh_private.param("amplitude" , amplitude, 0.0 );
    double frequency;
    nh_private.param("frequency" , frequency, 0.0 );
    double bias;
    nh_private.param("bias" , bias, 0.0 );
    double phase;
    nh_private.param("phase" , phase, 0.0 );
    double initial_time;
    nh_private.param("initial_time" , initial_time, 0.0 );

    string topic_output_str;
    nh_private.param("topic_output" , topic_output_str, string("sine_wave") );
    string output_type_str;
    nh_private.param("output_type" , output_type_str, string("std_msgs/Float64") );
    nh_private.param("msg_index" , msg_index, 0 );

    boost::function< void(double) > pub_fun;

    switch (str2int(output_type_str.c_str()))
    {
        case str2int("std_msgs/Float64"):{
            pub_out = nh_public.advertise<std_msgs::Float64>(topic_output_str, 1);
            pub_fun = pubFloat64;
            break;
        }
        case str2int("geometry_msgs/Wrench"):{
            pub_out = nh_public.advertise<geometry_msgs::Wrench>(topic_output_str, 1);
            pub_fun = pubWrench;
            break;
        }
        case str2int("geometry_msgs/WrenchStamped"):{
            pub_out = nh_public.advertise<geometry_msgs::WrenchStamped>(topic_output_str, 1);
            pub_fun = pubWrenchStamped;
            break;
        }
        default:{
            cout << HEADER_PRINT BOLDRED "ERROR: Non supported output type " << output_type_str << CRESET << endl;
            exit(-1);
        }
    }

    /*Build traj*/
    Sine_Traj sin_traj(
            duration,
            amplitude,
            frequency,
            bias,
            phase,
            initial_time
          );

    //while(!pub_out.isLatched());
    sleep(3);

    cout << HEADER_PRINT GREEN "Start!" CRESET << endl;
    ros::Rate loop_rate(hz);
    double time_now = ros::Time::now().toSec();

    sin_traj.changeInitialTime(time_now);

    while (ros::ok() && !sin_traj.isCompleate(time_now) )
    {

        time_now = ros::Time::now().toSec();

        pub_fun( sin_traj.getPosition(time_now) );

        loop_rate.sleep();

    }

    cout << HEADER_PRINT GREEN "END!" CRESET << endl;
    
    

    return 0;
}