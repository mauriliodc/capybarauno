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


// H E A D E R S
//
//

// project headers
#include "odom/capybarauno_odom.h"
#include "joy/capybarauno_move.h"


/// @class a class that combines the config for the odometry and the move/joystick classes
class CapybaraunoConfig : public JoyConfig, public OdomConfig {
	public:
		void printParameters() {
			JoyConfig::printParameters();
			OdomConfig::printParameters();
		}
};


/// @class interface class to use the robot: combines odometry and movement into a single class.
class CapybaraunoRobot {
	public:
		/// @brief constructor, that passes the config to the sub-components
		CapybaraunoRobot( CapybaraunoConfig config ) : config_(config), odom_(config_), move_(config_) {};
		
		/// @brief initializes the object. should only be called when the config has been fully populated
		void init() {
			// initializes the odometry and the move object, which sets up the serial communication with the microcontroller
			if( !move_.init() )
				exit( 0 );
			if( !odom_.init() )
				exit( 0 );
		}
		
		/// @brief returns the current odometry via references
		void getOdometry( double &x, double &y, double &theta ) {
			OdomPose2d pose = odom_.getPose();
			x = pose.x_;
			y = pose.y_;
			theta = pose.theta_;
		}
		
		/// @brief sends a 'speed' command to the microcontroller. the parameter tv is the translational velocity, rv is the rotation speed
		void setSpeed( double &tv, double &rv ) {
			move_.sendSpeedCmd( tv*1000, rv*1000 );
		}
		
		void spinOnce() {
			odom_.spinOnce();
		}
		
		void spin() {
			while( true ) {
				spinOnce();
			}
		}
		
	protected:
		/// @brief object holding the config for this class
		CapybaraunoConfig config_;
		/// @brief object to handle the odometry
		EncoderOdom odom_;
		/// @brief object to send movement commands to the robot
		CapybaraunoMove move_;
};







