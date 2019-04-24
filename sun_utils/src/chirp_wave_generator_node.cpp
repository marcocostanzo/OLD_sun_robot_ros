/*

    node that generates a chirp wave

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
#include "sun_robot_msgs/PoseTwistStamped.h"
#include "sun_robot_msgs/ClikStatus.h"

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

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "chirp_wave_generator");
    
    ros::NodeHandle nh_private = ros::NodeHandle("~");
    ros::NodeHandle nh_public = ros::NodeHandle();
    
    //params
    double hz;
    nh_private.param("hz" , hz, 300.0 );
    double duration;
    nh_private.param("duration" , duration, 300.0 );
    double amplitude;
    nh_private.param("amplitude" , amplitude, 0.0002 );
    double f0;
    nh_private.param("f0" , f0, 0.01 );
    double freq_k;
    nh_private.param("freq_k" , freq_k, 0.01 );
    double bias;
    nh_private.param("bias" , bias, 0.0 );
    double phase0;
    nh_private.param("phase0" , phase0, 0.0 );
    double initial_time;
    nh_private.param("initial_time" , initial_time, 0.0 );

    string topic_output_str;
    nh_private.param("topic_output" , topic_output_str, string("/sia5f/force_control/desired_pose_twist_in") );

    ros::Publisher pub = nh_public.advertise<sun_robot_msgs::PoseTwistStamped>(topic_output_str, 1);

    ros::ServiceClient srv_get_pos = nh_public.serviceClient<sun_robot_msgs::ClikStatus>("/sia5f/force_control/get_clik_status");

    //while(!pub_out.isLatched());
    sleep(3);

    srv_get_pos.waitForExistence();

    sun_robot_msgs::ClikStatus::Response res;
    sun_robot_msgs::ClikStatus::Request req;
    req.refresh = true;
    bool b_result = srv_get_pos.call(req,res);
    res.success = b_result && res.success;
    if(!res.success){
        cout << HEADER_PRINT BOLDRED "Failed to update clik status" CRESET << endl;
        return -1;
    }

    bias = bias + res.pose.position.z;

    sun_robot_msgs::PoseTwistStamped out_msg;

    out_msg.pose.position.x = res.pose.position.x;
    out_msg.pose.position.y = res.pose.position.y;
    out_msg.pose.position.z = res.pose.position.z;
    out_msg.pose.orientation.x = res.pose.orientation.x;
    out_msg.pose.orientation.y = res.pose.orientation.y;
    out_msg.pose.orientation.z = res.pose.orientation.z;
    out_msg.pose.orientation.w = res.pose.orientation.w;
    out_msg.twist.linear.x = 0.0;
    out_msg.twist.linear.y = 0.0;
    out_msg.twist.linear.z = 0.0;
    out_msg.twist.angular.x = 0.0;
    out_msg.twist.angular.y = 0.0;
    out_msg.twist.angular.z = 0.0; 

    cout << "Initial position :" << endl <<
    out_msg.pose.position.x << " " <<
    out_msg.pose.position.y << " " <<
    out_msg.pose.position.z << " " << endl <<
    "quat: " << endl <<
    out_msg.pose.orientation.w << " <" <<
    out_msg.pose.orientation.x << " " << 
    out_msg.pose.orientation.y << " " <<
    out_msg.pose.orientation.z << ">" << endl;

    cout << "continue?";
    char ans;
    cin >> ans;

    if( ans != 'y' )
        exit(-1);

    cout << HEADER_PRINT GREEN "Start!" CRESET << endl;
    ros::Rate loop_rate(hz);
    double t0 = ros::Time::now().toSec();

    double t = 0.0;

    while (ros::ok() &&  t < duration )
    {

        t = ros::Time::now().toSec() - t0;

        double out = 
            amplitude * sin( phase0 + 2.0*M_PI*( f0*t + freq_k/2.0 * pow(t,2) ) ) + bias;

        double dout = amplitude * cos( phase0 + 2.0*M_PI*( f0*t + freq_k/2.0 * pow(t,2) ) )
                        * 2.0*M_PI*( f0 + freq_k * t );

        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "chirp";
        out_msg.pose.position.z = out;
        out_msg.twist.linear.z = dout;

        pub.publish(out_msg);
        

        loop_rate.sleep();

    }

    out_msg.twist.linear.z = 0.0;
    pub.publish(out_msg);
    pub.publish(out_msg);
    pub.publish(out_msg);

    cout << HEADER_PRINT GREEN "END!" CRESET << endl;
    
    

    return 0;
}