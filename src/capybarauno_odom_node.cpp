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
 * @brief Ros node to provide an odometry topic in conjunction with the the Capybarauno robot's wheel encoders.
 * 
 * This node provides odometry and (optionally) tf frames for the Capybarauno robot. The wheel encoder states
 * are read through a serial interface (by default located at /dev/ttyACM0) and are translated into the
 * robots current position.
 */


// H E A D E R S
//
//

// project headers
#include "odom/capybarauno_odom.h"
#include "odom/capybarauno_odom_node.h"




// G L O B A L   V A R I A B L E S
//
//

// none




// F U N C T I O N S
//
//



int main( int argc, char **argv ) {
	
	// initialize ros for this node
	ros::init( argc, argv, "capybarauno_odom" );
	
	// create the encoder object and start spinning
	EncoderOdomNode eo_node;
	eo_node.spin();		// stops spinning when ros::ok() returns false

    return 0;
    
}


















