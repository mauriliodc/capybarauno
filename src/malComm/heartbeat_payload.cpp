#include "mal_comm.h"

extern struct Heartbeat heartbeat;

char* Heartbeat_Payload_write(const struct Packet* p, char* buffer, int ascii){
//BinaryMode
  if (!ascii) {
    uint8_t lenght = 7;
    //PACKET LENGTH
    buffer = writeUint8(lenght, buffer);
    //PACKET ID
    buffer = writeUint8(Heartbeat_Payload_ID, buffer);
    buffer = writeUint32(p->seq, buffer);
    buffer = writeUint16(p->heartbeat.beat, buffer);
  }//Ascii mode
    else {
        buffer = writeHeaderAscii(buffer);
        buffer = writeUint8Ascii(Heartbeat_Payload_ID, buffer);
        buffer = writeUint32Ascii(p->seq, buffer);
        buffer = writeUint16Ascii(p->heartbeat.beat, buffer);
        buffer = writeFooterAscii(buffer);
    }
    return buffer;
}
char* Heartbeat_Payload_read(struct Packet* p, char* buffer, int ascii){
 if (!ascii) {
        buffer = readUint32(&(p->seq), buffer);
        buffer = readUint16((uint16_t*)&(p->heartbeat.beat), buffer);
    } else {
        long int a, b = 0;
        sscanf(buffer, "%ld %ld", &a, &b);
        p->seq = (uint32_t) a;
        p->heartbeat.beat = (uint16_t) b;

    }
    return buffer;
}
void Heartbeat_Payload_execute(struct Packet* p){

}
