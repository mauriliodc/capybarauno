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
#include "serial_interface.h"


/// static variable, that contains that maps all open addresses to SerialFileHandle objects
static std::map<const char*, SerialFileHandle> file_list;


int SerialInterface::open( const char *addr ) {
	std::map<const char*, SerialFileHandle>::iterator it;
	it = file_list.find(addr);
	
	// check if the serial port has already been opened
	if( it != file_list.end() ) {
		it->second.n_open_++;
		return it->second.handle_;
	}
	
	// open the serial port, create the SerialFileHandle object and return the handle
	int fd = openPort( addr );
	if( fd == -1 ) {
		printf( "failed to open port '%s'\n", addr );
		return -1;
	}
	
	SerialFileHandle file;
	file.n_open_ = 1;
	file.handle_ = fd;
	std::pair<const char*, SerialFileHandle> obj( addr, file );
	file_list.insert( obj );
	
	return fd;
}


void SerialInterface::close( int fd ) {
	std::map<const char*, SerialFileHandle>::iterator it;
	
	// iterate over all entries in the map
	for( it=file_list.begin(); it!=file_list.end(); ++it ) {
		
		// check if the file handle matches
		if( it->second.handle_ == fd ) {
			
			// decrease the 'open' counter
			it->second.n_open_--;
			
			// check if this was the only reference. if so, we close the serial connection. otherwise we keep it open.
			if( !it->second.n_open_ ) {
				closePort( it->second.handle_ );		// close connection
				file_list.erase( it );					// remove from map
			}
			return;
		}
	}
}
		



