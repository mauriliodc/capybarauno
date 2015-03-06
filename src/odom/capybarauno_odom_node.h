/* Copyright (c) 2015, Lab Ro.Co.Co ( http://www.dis.uniroma1.it/~labrococo/ )
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


/** @copyright Lab Ro.Co.Co. 2015
 * 
 * @note BSD license (3-clause BSD, aka new BSD)
 * 
 * @author j. dichtl
 * 
 */


// I N C L U D E S
//
//

// project headers
#include "capybarauno_odom.h"

// ROS headers
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>




// C L A S S E S
//
//

/** @class object to handle the publishing of ros topics for the EncoderOdom object. also
 *         sets up the configuration object, based on the the set ROS parameters.
 */
class EncoderOdomNode {
	
	public:
		/// constructor that additionally calls ros::init
		EncoderOdomNode(  int argc, char **argv ) : config_(), encoder_odom_(config_) {
			ros::init( argc, argv, "capybarauno_odom_node" );
			nh_ = ros::NodeHandle( "~" );	// must not be called before ros::init(...)
			init();
		}
		
		/// constructor for the case that ros::init has already been called
		EncoderOdomNode() : nh_("~"), encoder_odom_(config_) {
			init();
		}
		
		/// calls spinOnce in a loop
		void spin() {
			//ros::ok() used to get the SIGINT ctrl+c
			while( ros::ok() ) {
				spinOnce();
				ros::spinOnce();
				usleep(1000);
			}
		}
		
		/// calls spinOnce of the EncoderOdom object, to check for encoder updates. publishes every n updates
		void spinOnce() {
			if( encoder_odom_.spinOnce() ) {	// true if we read a new packet via serial
				encoder_updates_++;
				if( encoder_updates_ >= config_.n_pub_updates_ ) {
					publish();
					encoder_updates_ = 0;
				}
			}
		}
		
		/// publishes the odom topic and (optionally) the tf topic
		void publish() {
			geometry_msgs::TransformStamped tf_msg;
			nav_msgs::Odometry odom_msg;
			OdomPose2d odom_pose = encoder_odom_.getPose();
			
			ros::Time now;
			now = ros::Time::now();
			ros::Duration dt = now - last_update_;
			last_update_ = now;
			
			// a quaternion, created from yaw
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw( odom_pose.theta_ );
			
			// create & publish the tf message
			//
			if( config_.publish_tf_ ) {
				tf_msg.header.stamp = now;
				tf_msg.header.frame_id = "odom";
				tf_msg.child_frame_id= "base_link";
				
				tf_msg.transform.translation.x = odom_pose.x_;
				tf_msg.transform.translation.y = odom_pose.y_;
				tf_msg.transform.translation.z = 0.0;
				tf_msg.transform.rotation = odom_quat;
				
				tf_pub_.sendTransform( tf_msg );
			}
			
			// create and publish the odom message
			//
			
			// set the message header
			odom_msg.header.frame_id = "odom";
			odom_msg.child_frame_id = "base_link";
			odom_msg.header.stamp = now;
			
			// set the position
			odom_msg.pose.pose.position.x = odom_pose.x_;
			odom_msg.pose.pose.position.y = odom_pose.y_;
			odom_msg.pose.pose.position.z = 0.0;
			odom_msg.pose.pose.orientation = odom_quat;
			
			// set valocity
			double it = 1.0 / dt.toSec();
			odom_msg.twist.twist.linear.x = (odom_pose.x_ - last_pose_.x_) * it;
			odom_msg.twist.twist.linear.y = (odom_pose.y_ - last_pose_.y_) * it;
			odom_msg.twist.twist.angular.z = 0.0;		// todo: compute this
			
			last_pose_ = odom_pose;
			
			odom_pub_.publish( odom_msg );
		}
	
	
	protected:
		
		/// initializes the object
		void init() {
			// read parameters and set defaults when needed
			nh_.param( "wheel_radius", config_.wheel_radius_, 0.05 );						// default: radius=5cm
			nh_.param( "encoder_count", config_.n_encoder_, 3600 );							// default: 10 ticks per 1.0 degree
			nh_.param( "wheel_distance", config_.wheel_distance_, 0.2 );					// default: 20cm distance between the wheels
			nh_.param<std::string>( "odom_topic", config_.odom_topic_, "odom" );			// unused at the moment
			nh_.param<std::string>( "comm_port", config_.comm_port_, "/dev/ttyACM0" );		// address of the serial port
			nh_.param( "comm_ascii", config_.comm_ascii_, 0 );								// ascii(=1) or binary(=0) communication on the serial
			nh_.param( "n_pub_updates", config_.n_pub_updates_, 50 );						// number of encoder updates read before we publish the ros topic
			nh_.param( "publish_tf", config_.publish_tf_, 0 );								// if non-zero we publish the tf topic as well
			nh_.param( "debug", config_.debug_, 1 );
			
			// publish the odomerty with a queue size of 20
			odom_pub_ = nh_.advertise< nav_msgs::Odometry >( config_.odom_topic_.c_str(), 20 );
			
			// print configuration to the screen
			config_.printParameters();
			
			encoder_odom_.init();
		}
		
		/// node handle that is used in this class.
		ros::NodeHandle nh_;
		/// configuration object
		OdomConfig config_;
		/// object that takes care of reading the encoder data from the serial connection, and computes the odom
		EncoderOdom encoder_odom_;
		/// odometry publisher
		ros::Publisher odom_pub_;
		/// tf broadcaster
		tf::TransformBroadcaster tf_pub_;
		/// number of encoder updates since the last time we published
		unsigned int encoder_updates_;
		/// timestamp when we last published the odometry topic
		ros::Time last_update_;
		/// the 2d pose that we received for the last update. required to compute velocities
		OdomPose2d last_pose_;
};



















