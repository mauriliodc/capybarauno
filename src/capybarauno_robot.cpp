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
#include "capybarauno_robot.h"
#include "stdio.h"
#include <unistd.h>			// for usleep (test-code)
#include "serial_interface.h"







int main( int argc, char **argv ) {
	CapybaraunoConfig config;
	
	// set default values in the config
	config.JoyConfig::comm_port_ = config.OdomConfig::comm_port_ = "/dev/ttyACM0";
	config.JoyConfig::comm_ascii_ = config.OdomConfig::comm_ascii_ = 0;
	config.JoyConfig::debug_ = config.OdomConfig::debug_ = 1;
	
	config.n_encoder_ = 3600;		// 10 encoder readings for every degree
	config.wheel_radius_ = 0.05;	// 5 cm wheel radius
	config.wheel_distance_ = 0.4;	// 40 cm distance between the wheels
	config.odom_topic_ = "";		// not runnning as a ROS node, so no topic name
	config.n_pub_updates_ = 0;		// not relevant, because we are not running as a ROS node
	config.publish_tf_ = 0;			// not relevant, because we are not running as a ROS node
	
	config.trans_axis_ = 0;			// ignored, because we are not reading joystick input
	config.rot_axis_ = 0;			// ignored, because we are not reading joystick input
	config.boost_button_ = 0;		// ignored, because we are not reading joystick input
	config.stop_button_ = 0;		// ignored, because we are not reading joystick input
	config.dead_man_button_ = 0;	// ignored, because we are not reading joystick input
	config.trans_multiplier_ = 1.0;	// multiplier to adjust speed commands to your robot (translational movement speed)
	config.rot_multiplier_ = 1.0;	// multiplier to adjust speed commands to your robot (rotation speed)
	config.boost_multiplier_ = 2.0;	// ignored, because we are not reading joystick input
	config.joy_topic_ = "";			// ignored, because we are not running as a ROS node
	
	// read command line options
	for( int i=0; i<argc; i++ ) {
		if( !strcmp(argv[i], "comm_port") ) {
			i++;
			config.JoyConfig::comm_port_ = argv[i];
			config.OdomConfig::comm_port_ = argv[i];
		} else if( !strcmp(argv[i], "comm_ascii") ) {
			i++;
			config.JoyConfig::comm_ascii_ = atoi( argv[i] );
			config.OdomConfig::comm_ascii_ = atoi( argv[i] );
		} else if( !strcmp(argv[i], "n_encoder") ) {
			i++;
			config.n_encoder_ = atoi( argv[i] );
		} else if( !strcmp(argv[i], "wheel_radius") ) {
			i++;
			config.wheel_radius_ = atof( argv[i] );
		} else if( !strcmp(argv[i], "trans_multiplier") ) {
			i++;
			config.trans_multiplier_ = atof( argv[i] );
		} else if( !strcmp(argv[i], "rot_multiplier") ) {
			i++;
			config.rot_multiplier_ = atof( argv[i] );
		} else if( !strcmp(argv[i], "debug") ) {
			i++;
			config.JoyConfig::comm_ascii_ = config.OdomConfig::comm_ascii_ = atoi( argv[i] );
		}
	}
	
	config.printParameters();
	
	// create the robot object and initialize it
	CapybaraunoRobot robot( config );
	robot.init();
	
	// test code
	//
	double x=0.0, y=0.0, theta=0.0;		// current position
	double tv=0.0, rv=0.0;				// current requested speed
	int a=500, b=200;					// counters
	while( true ) {
		a++;
		b++;
		robot.getOdometry( x, y, theta );
		
		double t = (double)((a%1000)-500) / 8000000.0;
		tv += t;
		t = (double)((b%400)-200) / 20000000.0;
		rv += t;
		
		// cap max speed
		if( tv > 0.5 )
			tv = 0.5;
		if( tv < -0.5 )
			tv = -0.5;
		if( rv > 0.5 )
			rv = 0.5;
		if( rv < -0.5 )
			rv = -0.5;
		
		robot.setSpeed( tv, rv );
		printf( "\rodom: <%f, %f, %f>, speed: <%f %f>            ", x, y, theta, tv, rv );
		for( int i=0; i<2; i++ ) {
			robot.spinOnce();
			usleep( 1000 );
		}
	}
	//
	// end of test code
}























