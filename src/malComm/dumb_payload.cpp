#include "mal_comm.h"
//DumbPacket
//==============================================================================
//Write
//==============================================================================
char* Dumb_Payload_write(const struct Packet* p, char* buffer, int ascii) {
  if (!ascii) {

  } else {

  }
  return buffer; //DUMMY
}

//Read
//==============================================================================
char* Dumb_Payload_read(struct Packet* p, char* buffer, int ascii) {
  return buffer;
}

void Dumb_Payload_execute(struct Packet* p){
}
//==============================================================================
//==============================================================================
