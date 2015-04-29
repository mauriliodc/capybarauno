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
#include <math.h>			// for M_PI constant only



// C O N S T A N T S
//
//

// none yet




// C L A S S E S
//
//


/** @class class to put all the parameters for the project into a single object. */
class OdomConfig {
	public:
		/// number of encoder readings for a full rotation, i.e. how many encoder ticks translate to a 360 degree rotation.
		int n_encoder_;
		/// radius of the wheels
		double wheel_radius_;
		/// distance between the two wheels
		double wheel_distance_;
		/// name of the published odomerty topic, e.g. "odom"
		std::string odom_topic_;
		/// serial port, e.g. "/dev/ttyACM0"
		std::string comm_port_;
		/// set 1/true if serial communication is in ascii format
		int comm_ascii_;
		/// number of encoder updates read before we publish on ros. set to 0 or 1 to update on every encoder update.
		int n_pub_updates_;
		/// if non-zero we publish the tf topic as well
		int publish_tf_;
		/// if non-zero, we print a few more messages
		int debug_;
		
		/// prints the config to the console
		void printParameters( void ) {
			printf( "odom. configuration:\n" );
			printf( "  %s %i\n", "n_encoder", n_encoder_ );
			printf( "  %s %f\n", "wheel_radius", wheel_radius_ );
			printf( "  %s %f\n", "wheel_distance", wheel_distance_ );
			printf( "  %s %s\n", "odom_topic", odom_topic_.c_str() );
			printf( "  %s %s\n", "comm_port", comm_port_.c_str() );
			printf( "  %s %i\n", "comm_ascii", comm_ascii_ );
			printf( "  %s %i\n", "n_pub_updates", n_pub_updates_ );
			printf( "  %s %i\n", "publish_tf", publish_tf_ );
			printf( "  %s %i\n", "debug", debug_ );
		}
};


/** @class class to pass the 2d odometry data between objects. */
class OdomPose2d {
	public:
		/** @brief default constructor, setting all members to 0.0 */
		OdomPose2d() : x_(0.0), y_(0.0), theta_(0.0) {}
		/** @brief constructor that sets the members to the provided values */
		OdomPose2d( double x, double y, double theta ) : x_(x), y_(y), theta_(theta) {}
		
		double x_;
		double y_;
		double theta_;
};


/** @class class to handle the serial communication with the microcontroller and to compute
 *         the robot pose, based on the encoder readings
 * 
 * @note we assume two identical wheels (same encoder count and identical wheel radius)
 */
class EncoderOdom {
	
	public:
		/// @brief constructor. the cfg object must be set before we can call init()
		EncoderOdom( OdomConfig &cfg ) : config_(cfg) {
			// init is not called here, because in general the OdomConfig object 'cfg' is not fully set when this constructor is called
		}
		
		/// @brief initializes the object. called by all constructors.
		bool init( void ) {
			// set encoder ticks to zero
			old_ticks_left_ = 0;
			old_ticks_right_ = 0;
			
			// set starting position to (0,0,0 , 0,0,0)
			odom_x_ = 0.0;
			odom_y_ = 0.0;
			odom_theta_ = 0.0;
			
			// setup the serial connection
			return initComm();
		}
		
		/** @brief performs all the actions for a single spin, i.e. read from uart, compute odometry on
		 *         updates, and publishes the odom topic periodically.
		 * 
		 * @return 0	no update (did not receive a full packet via serial)
		 * @return 1	update performed (a new packet has been received and parsed)
		 */
		int spinOnce( void ) {
			Packet robot_data;
			
			if( readFromUart(robot_data) ) {
				update( robot_data );
				return 1;
			}
			
			return 0;
		}
		
		/** @brief returns the current pose of the robot via a OdomPose2d object
		 */
		OdomPose2d getPose() {
			/// todo: add velocities, too
			OdomPose2d pose( odom_x_, odom_y_, odom_theta_ );
			return pose;
		}
	
	
	protected:
		/** @brief initializes the serial communication
		 * 
		 * @return true if the serial port has been opened
		 * @return false if the serial port could not be opened
		 */
		bool initComm( void ) {
			initConsts();
			Packet_Decoder_init( &packet_decoder_, config_.comm_ascii_ );
			//serial_fd_ = openPort( config_.comm_port_.c_str() );
			serial_fd_ = SerialInterface::open( config_.comm_port_.c_str() );
			
			if( serial_fd_ == -1 ) {
				printf( "ERROR: failed to open port '%s'\n", config_.comm_port_.c_str() );
			}
			
			if( config_.debug_ || 1 ) {
				printf( "serial port status: %d\n", serial_fd_ );
			}
			
			return (serial_fd_ == -1) ?  false : true;
		}
		
		/// reads from the serial interface. if a packet has been fully read and parsed, we return 1, otherwise we return 0.
		int readFromUart( Packet &packet ) {
			char c;
			int complete = 0;
			
			// read from the serial port until we have read a complete package that can be parsed, or we no longer read a character
			while( read(serial_fd_, &c, 1) > 0  &&  !complete ) {
				complete = Packet_Decoder_putChar( &packet_decoder_,(unsigned char)c );
			}
			
			// read the packet
			if( complete ) {
				Packet_parse( packet_decoder_.buffer_start, &packet, config_.comm_ascii_ );
			}
			
			return complete;
		}
		
		/// updates the internal state, according to the data in the packet
		void update( const Packet &packet ) {
			// read the current encoder state
			uint16_t ticks_left = packet.state.leftEncoder;
			uint16_t ticks_right = packet.state.rightEncoder;
			
			// compute the difference between last time and now
			int delta_left = (int16_t)(ticks_left - old_ticks_left_);
			int delta_right = (int16_t)(ticks_right - old_ticks_right_);
			
			// check for unexpected large values (note: this is not uncommen for the first reading upon startup)
			if( delta_left > 200 || delta_left < -200  ||  delta_right > 200 || delta_right < -200 ) {
				/// todo: find out what causes these unexpected values that sometimes appear at the second update
				printf( "WARNING: unexpected large value(s): delta-left=%i, delta-right=%i     \n", delta_left, delta_right );
				//return;
			}
			
			// save current state as new 'old state'
			old_ticks_left_ = ticks_left;
			old_ticks_right_ = ticks_right;
			
			double factor = 2 * M_PI * config_.wheel_radius_ / config_.n_encoder_;
			
			// compute the movement. note: this is just an aproximation
			double dwl = delta_left * factor;					// delta movement in meter for the left wheel
			double dwr = delta_right * factor;					// delta movement in meter for the right wheel
			double ds = ( dwl + dwr) / 2;						// delta s (distance moved)
			double dtheta = (dwl-dwr) / config_.wheel_distance_;// delta theta (rotation)
			
			// save in the objects member variables
			odom_theta_ += dtheta;
			odom_x_ += ds * cos(odom_theta_ );
			odom_y_ += ds * sin( odom_theta_ );
			
			// for debugging (note: spamming a lot. writes to the same line over and over again)
			//printf( "\r  left=%i, right=%i,   delta-left=%i, delta-right=%i             ", (int)ticks_left, (int)ticks_right, (int)delta_left, (int)delta_right );
		}
		
		
		/// configuration object
		OdomConfig &config_;
		/// serial connection file descriptor
		int serial_fd_;
		/// structure to store incomplete packets (read from serial) until we can parse them
		struct Packet_Decoder packet_decoder_;
		/// odometry: x-position
		double odom_x_;
		/// odometry: y-position
		double odom_y_;
		/// odometry: orientation (yaw-angle)
		double odom_theta_;
		/// encoder state from the last received packet (left encoder)
		uint16_t old_ticks_left_;
		/// encoder state from the last received packet (right encoder)
		uint16_t old_ticks_right_;
	
};


















