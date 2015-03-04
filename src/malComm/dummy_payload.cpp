#include "mal_comm.h"
//==============================================================================
//DummyPacket
//==============================================================================
//pay attention, this make side effect on buffer
//The checksum is computed on the first byte of each field.
//We need to get the addresws of (buffer-1) because the Write primitive does side effect
//on the buffer, so the returned buffer points to a NUL byte
char* Dummy_Payload_read(struct Packet* p, char* buffer, int ascii) {
  //Don't have to read the lenght
  if (!ascii) {
    buffer = readUint32(&(p->seq), buffer);
    buffer = readUint8(&(p->dummy.field_1), buffer);
    buffer = readUint16(&(p->dummy.field_2), buffer);
    buffer = readUint8((uint8_t*)&(p->dummy.field_3), buffer);
    buffer = readUint8((uint8_t*)&(p->dummy.field_4), buffer);
    buffer = readFloat(&(p->dummy.field_5), buffer);
    buffer = readUint8(&(p->dummy.field_6), buffer);
    buffer = readUint32(&(p->dummy.field_7), buffer);
    buffer = readUint32((uint32_t*)&(p->dummy.field_8), buffer);
  } else {

    //clean this mess, someday
    long int a, b, c, d, e, g, h, i = 0;
    float f = 0;
    sscanf(buffer, "%ld %ld %ld %ld %ld %f %d %ld %ld",
    &a,
    &b,
    &c,
    &d,
    &e,
    &f,
    &g,
    &h,
    &i);

    p->seq = (uint32_t) a;
    p->dummy.field_1 = (uint8_t) b;
    p->dummy.field_2 = (uint16_t) c;
    p->dummy.field_3 = (int8_t) d;
    p->dummy.field_4 = (char) e;
    p->dummy.field_5 = (float) f;
    p->dummy.field_6 = (unsigned char) g;
    p->dummy.field_7 = (uint32_t) h;
    p->dummy.field_8 = (int32_t) i;

  }
  return buffer;

}

char* Dummy_Payload_write(const struct Packet* p, char* buffer, int ascii) {
  //BinaryMode
  if (!ascii) {
    uint8_t lenght = 23;
    //PACKET LENGTH
    buffer = writeUint8(lenght, buffer);
    //PACKET ID
    buffer = writeUint8(Dummy_Payload_ID, buffer);
    buffer = writeUint32(p->seq, buffer);

    buffer = writeUint8(p->dummy.field_1, buffer);
    buffer = writeUint16(p->dummy.field_2, buffer);
    buffer = writeUint8(p->dummy.field_3, buffer);
    buffer = writeChar(p->dummy.field_4, buffer);
    buffer = writeFloat(p->dummy.field_5, buffer);
    buffer = writeChar(p->dummy.field_6, buffer);
    buffer = writeUint32(p->dummy.field_7, buffer);
    buffer = writeUint32(p->dummy.field_8, buffer);
  }//Ascii mode
  else {
    buffer = writeHeaderAscii(buffer);
    buffer = writeUint8Ascii(Dummy_Payload_ID, buffer);
    buffer = writeUint32Ascii(p->seq, buffer);
    buffer = writeUint8Ascii(p->dummy.field_1, buffer);
    buffer = writeUint16Ascii(p->dummy.field_2, buffer);
    buffer = writeUint8Ascii(p->dummy.field_3, buffer);
    buffer = writeCharAscii(p->dummy.field_4, buffer);
    buffer = writeFloatAscii(p->dummy.field_5, buffer);
    buffer = writeCharAscii((char) p->dummy.field_6, buffer);
    buffer = writeUint32Ascii(p->dummy.field_7, buffer);
    buffer = writeUint32Ascii(p->dummy.field_8, buffer);
    buffer = writeFooterAscii(buffer);
  }
  return buffer;
}

void Dummy_Payload_execute(struct Packet* p){

}
//==============================================================================
//==============================================================================
