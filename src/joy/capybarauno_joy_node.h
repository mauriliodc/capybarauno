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
#include "capybarauno_joy.h"

// ROS headers
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"




// C L A S S E S
//
//



/** @class ros node to forward joystick commands as movement commands to the micro controller. */
class CapybaraunoJoy {
	public:
		CapybaraunoJoy() : nh_("~"), move_(config_) {
			init();
		}
		void spin( void ) {
			while( ros::ok() ) {
				/// todo: check when we last received the button-pressed message from the dead man switch and stop the robot if necessary.
				ros::spinOnce();
				usleep( 1000 );
			}
		}
		
	protected:
		
		/// initializes the object
		void init();
		/// called when we receive a joystick message
		void joyCallback( const sensor_msgs::Joy::ConstPtr& joy ) {
			// debug message
			printf( "." );
			
			// get absolute speed values, expressed in tick per interval
			double trans_vel = joy->axes[config_.trans_axis_] * config_.trans_multiplier_;
			double rot_vel = joy->axes[config_.rot_axis_] * config_.rot_multiplier_;
			
			// check if boost button is pressed
			if( joy->buttons[config_.boost_button_] == 1 ) {
				trans_vel *= config_.boost_multiplier_;
				rot_vel *= config_.boost_multiplier_;
			}
			
			// now check if the stop button is pressed to halt the robot
			/// todo: gradualy halt over a fixed time frame, instead of trying a full stop at an instant
			if( joy->buttons[config_.stop_button_] == 1 ){
				trans_vel = 0;
				rot_vel = 0;
			}
			
			move_.sendSpeedCmd( trans_vel, rot_vel );
		}
		
		/// node handle that is used in this class.
		ros::NodeHandle nh_;
		/// configuration object
		JoyConfig config_;
		/// object to send the move commands to the micro controller
		CapybaraunoMove move_;
		/// ros subscriber to joystick topic
		ros::Subscriber joy_sub_;
};




























