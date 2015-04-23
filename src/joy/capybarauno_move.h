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


#pragma once

// H E A D E R S
//
//

// project headers
//#include "../malComm/mal_comm.h"
//#include "../malComm/mal_primitives.h"
#include "../serial_interface.h"


// C++ headers
#include <string>
//#include <math.h>			// for M_PI constant only



// C O N S T A N T S
//
//

// none yet




// C L A S S E S
//
//


/** @class class to put all the parameters for the project into a single object. */
class JoyConfig {
	public:
		/// joystick axis for translational commands (forward/backward)
		int trans_axis_;
		/// joystick axis for rotational commands (turn left/right)
		int rot_axis_;
		/// button for speed boost ("turbo button")
		int boost_button_;
		/// button for a full stop
		int stop_button_;
		/// dead_man_button (unused at the moment)
		int dead_man_button_;
		/// multiplier for translational movement commands
		double trans_multiplier_;
		/// multiplier for rotational movement commands
		double rot_multiplier_;
		/// multiplier for translational commands when the boost button is held
		double boost_multiplier_;
		/// topic name to subscribe at for joystick messages
		std::string joy_topic_;
		/// serial port, e.g. "/dev/ttyACM0"
		std::string comm_port_;
		/// set to 1 if serial communication is in ascii format, and to 0 for binary format
		int comm_ascii_;
		/// if non-zero, we print a few more messages to the console
		int debug_;
		
		/// prints the config to the console
		void printParameters( void ) {
			printf( "joy. configuration:\n" );
			printf( "  %s %i\n", "trans_axis", trans_axis_ );
			printf( "  %s %i\n", "rot_axis", rot_axis_ );
			printf( "  %s %i\n", "boost_button", boost_button_ );
			printf( "  %s %i\n", "stop_button", stop_button_ );
			printf( "  %s %i\n", "dead_man_button", dead_man_button_ );
			
			printf( "  %s %f\n", "trans_multiplier", trans_multiplier_ );
			printf( "  %s %f\n", "rot_multiplier", rot_multiplier_ );
			printf( "  %s %f\n", "boost_multiplier", boost_multiplier_ );
			
			printf( "  %s %s\n", "joy_topic", joy_topic_.c_str() );
			printf( "  %s %s\n", "comm_port", comm_port_.c_str() );
			
			printf( "  %s %i\n", "comm_ascii", comm_ascii_ );
			printf( "  %s %i\n", "debug", debug_ );
		}
};


/** @class this class is dedicated to sending velocity commands via serial interface to the micro controller.
 */
class CapybaraunoMove {
	public:
		/// @brief constructor, that expects that ros::init has already been called
		CapybaraunoMove( JoyConfig &cfg ) : config_(cfg) {
			//init();
		}
		
		/** @brief should be called when the config has been populated. initializes the object, including
		 *         opening the serial port.
		 */
		bool init() {
			return initComm();
		}
		
		/** @brief this function takes the translational and rotaional velocities and sends the "speed"
		 *         command via the serial interface.
		 */
		void sendSpeedCmd( double trans_vel, double rot_vel ) {
			struct Speed_Payload payload;
			
			// compose the payload. note the implicit typecast from double to int16_t.
			payload.leftTick = -trans_vel + rot_vel;
			payload.rightTick = trans_vel + rot_vel;
			
			//assign the payload to the general packet
			packet_.speed = payload;
			
			// write the packet to the provided buffer
			char buffer[255];
			char* end_ptr = Packet_write( &packet_, buffer, config_.comm_ascii_ );
			
			// send the packet from the buffer
			sendToUart( serial_fd_, buffer, end_ptr-buffer, 0 );
			
			if( config_.debug_ ) {
				if( config_.comm_ascii_ == 1 ) {
					/// todo: check, if this is save for ascii==0. maybe write a function to print in HEX for ascii==0 ...
					printf( "SENDING: %s\n", buffer );
				} else {
					printf( "SENDING (binary) <%+03i / %+03i>\n", payload.leftTick, payload.rightTick );
				}
			}
		}
		
		
		
	protected:
		/** @brief initializes the serial communication
		 * 
		 * @return true if the serial port has been opened
		 * @return false if the serial port could not be opened
		 */
		bool initComm( void ) {
			initConsts();
			
			//serial_fd_ = openPort( config_.comm_port_.c_str() );
			serial_fd_ = SerialInterface::open( config_.comm_port_.c_str() );
			
			if( serial_fd_ == -1 ) {
				printf( "ERROR: failed to open port '%s'\n", config_.comm_port_.c_str() );
			}
			
			if( config_.debug_ || 1 ) {
				printf( "serial port status: %d\n", serial_fd_ );
			}
			
			// set the packet id to indicate that it holds speed commands
			packet_.id = Speed_Payload_ID;

			return (serial_fd_ == -1) ? false : true;
		}
		
		/// configuration object
		JoyConfig &config_;
		/// serial connection file descriptor
		int serial_fd_;
		/// packet that is send via serial port. used to send speed commands
		Packet packet_;
		
};




















