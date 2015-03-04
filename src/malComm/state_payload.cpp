#include "mal_comm.h"


//SlimStatePacket
//==============================================================================
//Write
//==============================================================================
char* State_Payload_write(const struct Packet* p, char* buffer, int ascii) {
  //BinaryMode
  if (!ascii) {
    uint8_t lenght = 9;
    //PACKET LENGTH
    buffer = writeUint8(lenght, buffer);
    //PACKET ID
    buffer = writeUint8(State_Payload_ID, buffer);
    buffer = writeUint32(p->seq, buffer);
    buffer = writeUint16(p->state.leftEncoder, buffer);
    buffer = writeUint16(p->state.rightEncoder, buffer);
  }//Ascii mode
    else {
        buffer = writeHeaderAscii(buffer);
        buffer = writeUint8Ascii(State_Payload_ID, buffer);
        buffer = writeUint32Ascii(p->seq, buffer);
        buffer = writeUint16Ascii(p->state.leftEncoder, buffer);
        buffer = writeUint16Ascii(p->state.rightEncoder, buffer);
        buffer = writeFooterAscii(buffer);
    }
    return buffer;
}

//Read
//==============================================================================

char* State_Payload_read(struct Packet* p, char* buffer, int ascii) {
    if (!ascii) {
        buffer = readUint32(&(p->seq), buffer);
        buffer = readUint16((uint16_t*)&(p->state.leftEncoder), buffer);
        buffer = readUint16((uint16_t*)&(p->state.rightEncoder), buffer);
    } else {
        long int a, b, c = 0;
        sscanf(buffer, "%ld %ld %ld", &a, &b, &c);
        p->seq = (uint32_t) a;
        p->state.leftEncoder = (uint16_t) b;
        p->state.rightEncoder = (uint16_t) c;
        
    }
    return buffer;
}

void State_Payload_execute(struct Packet* p){


}
//==============================================================================
//==============================================================================
