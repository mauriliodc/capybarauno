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
 * @brief ...
 * 
 * ... ... ...
 */


// H E A D E R S
//
//

// project headers
#include "joy/capybarauno_joy.h"
#include "joy/capybarauno_joy_node.h"




// G L O B A L   V A R I A B L E S
//
//

// none




// F U N C T I O N S
//
//



void CapybaraunoJoy::init() {
	// read parameters and set defaults when needed
	nh_.param( "trans_axis", config_.trans_axis_, 1 );					// default: 1 (= left joystick, up/down axis)
	nh_.param( "rot_axis", config_.rot_axis_, 2 );						// default: 2 (= right joystick, left/right axis)
	nh_.param( "boost_button", config_.boost_button_, 5 );				// default: 5 ...
	nh_.param( "stop_button", config_.stop_button_, 4 );				// default: 4 ...
	nh_.param( "dead_man_button", config_.dead_man_button_, -1 );		// default: -1 (= don't use this feature/button)
	
	nh_.param( "trans_multiplier", config_.trans_multiplier_, 10.0 );	// default: 
	nh_.param( "rot_multiplier", config_.rot_multiplier_, 5.0 );		// default: 
	nh_.param( "boost_multiplier", config_.boost_multiplier_, 2.0 );	// default: 2.0: twice the normal (translational) speed
	
	nh_.param<std::string>( "joy_topic", config_.joy_topic_, "/joy" );	// name of the joystick topic that we subscripe to
	
	nh_.param<std::string>( "comm_port", config_.comm_port_, "/dev/ttyACM0" );		// address of the serial port
	nh_.param( "comm_ascii", config_.comm_ascii_, 0 );								// ascii(=1) or binary(=0) communication on the serial
	
	nh_.param( "debug", config_.debug_, 0 );
	
	// subscribe to joystick topic
	printf( "subscribing to topic '%s'\n", config_.joy_topic_.c_str() );
	joy_sub_ = nh_.subscribe< sensor_msgs::Joy >( config_.joy_topic_.c_str(), 50, &CapybaraunoJoy::joyCallback, this );
	
	// print configuration to the screen
	config_.printParameters();
	
	// initialize the CapybaraunoMove object (needs to be done after config_ has been set up)
	move_.init();
}



int main( int argc, char **argv ) {
	
	// initialize ros for this node
	ros::init( argc, argv, "capybarauno_joy" );
	
	// create the JoyNode and start spinning
	CapybaraunoJoy joy_node;
	joy_node.spin();		// stops spinning when ros::ok() returns false

    return 0;
    
}


















