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
#include "malComm/mal_comm.h"
#include "malComm/mal_primitives.h"

// C++ headers
#include <map>




/** @class this class holds information about the open file (a 'reference counter' and the handle/identifier) */
class SerialFileHandle {
	public:
		/// how many times the file has been opened without being closed
		int n_open_;
		/// the handle/identifier that was returned by the posix open() function
		int handle_;
};



/** @class this class returns handles to the file (serial port) via the open() method. it allows multiple connections to a single serial port. it does however not
 *         manage the read/write operations, so in a typical scenario, you would want at most one read-only connection opened and one write-only connection opened,
 *         since read and write act like a seek operation, incrementing the index.
 */
class SerialInterface {
	public:
		/// opens a serial connection if it is not already open. it then returns the file handle of the open serial connection. returns -1 on errors.
		static int open( const char *addr );
		
		/// reduces the 'open' count on the open serial connection. closes the connection, if the 'open' count reaches zero.
		static void close( int fd );
		
	protected:
};

























