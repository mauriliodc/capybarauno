#include "mal_comm.h"


#include <stdio.h>
#include <stdlib.h>

#ifndef TOPORAGNAROK
#include "motor_controller.h"
extern struct MotorController LeftMotorController;
extern struct MotorController RightMotorController;
#endif

//SpeedStatePacket
//==============================================================================
//Write
//==============================================================================
char* Speed_Payload_write(const struct Packet* p, char* buffer, int ascii) {
  //BinaryMode
  if (!ascii) {
    uint8_t lenght = 9;
    //PACKET LENGTH
    buffer = writeUint8(lenght, buffer);
    //PACKET ID
    buffer = writeUint8(Speed_Payload_ID, buffer);
    buffer = writeUint32(p->seq, buffer);

    buffer = writeUint16(p->speed.leftTick, buffer);
    buffer = writeUint16(p->speed.rightTick, buffer);
  }//Ascii mode
    else {
        buffer = writeHeaderAscii(buffer);
        buffer = writeUint8Ascii(Speed_Payload_ID, buffer);
        buffer = writeUint32Ascii(p->seq, buffer);
        buffer = writeUint16Ascii(p->speed.leftTick, buffer);
        buffer = writeUint16Ascii(p->speed.rightTick, buffer);
        buffer = writeFooterAscii(buffer);
    }
    return buffer;
}

//Read
//==============================================================================

char* Speed_Payload_read(struct Packet* p, char* buffer, int ascii) {
    if (!ascii) {
        buffer = readUint32(&(p->seq), buffer);
        buffer = readUint16((uint16_t*)&(p->speed.leftTick), buffer);
        buffer = readUint16((uint16_t*)&(p->speed.rightTick), buffer);
    } else {
        long int a, b, c = 0;
        sscanf(buffer, "%ld %ld %ld", &a, &b, &c);
        p->seq = (uint32_t) a;
        p->speed.leftTick = (uint16_t) b;
        p->speed.rightTick = (uint16_t) c;
    }
    return buffer;

}

#ifndef TOPORAGNAROK
void Speed_Payload_execute(struct Packet* p){
    MotorController_setDesiredSpeed(&LeftMotorController,p->speed.leftTick);
    MotorController_setDesiredSpeed(&RightMotorController,p->speed.rightTick);
}
#else
void Speed_Payload_execute(struct Packet* p){

}
#endif
//==============================================================================
//==============================================================================
