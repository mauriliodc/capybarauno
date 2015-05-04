#include "mal_comm.h"

//INITIALIZATION STUFF
//==============================================================================
uint8_t Dummy_Payload_ID;
uint8_t Dumb_Payload_ID;
uint8_t State_Payload_ID;
uint8_t Init_Payload_ID;
uint8_t Speed_Payload_ID;
uint8_t Heartbeat_Payload_ID;
char packet_decoder_binary_header_1;
char packet_decoder_binary_header_2;
char packet_decoder_ascii_header;
char packet_decoder_ascii_footer;
int complete = 0;
//Consts init
//==============================================================================

void initConsts() {
    Dummy_Payload_ID = 1;
    Dumb_Payload_ID = 2;
    State_Payload_ID = 3;
    Init_Payload_ID = 4;
    Speed_Payload_ID = 5;
    Heartbeat_Payload_ID = 6;
    packet_decoder_binary_header_1 = 0xaa;
    packet_decoder_binary_header_2 = 0x55;
    packet_decoder_ascii_header = '(';
    packet_decoder_ascii_footer = ')';
}
//==============================================================================
//==============================================================================


//Decoder init and useful stuff
//==============================================================================

void Packet_Decoder_init(struct Packet_Decoder* d, int ascii) {
    Packet_Decoder_resetBuffers(d);
    d->buffer_start = &(d->buffer[0]);
    d->bufferPtr = &(d->buffer[0]);
    d->status = Unsync;
    d->lenght = 0;
    d->ascii = ascii;

}

void Packet_Decoder_resetBuffers(struct Packet_Decoder* d) {
    memset(d->buffer, '\0', BUFFER_SIZE);
    d->bufferPtr = d->buffer_start;
}
//==============================================================================
//==============================================================================

//Decoder main loop
//==============================================================================

int Packet_Decoder_putChar(struct Packet_Decoder* d, char c) {
    //binary mode
    //-----------------------------------------------------
    if (d->ascii == 0) {
        if (d->status == Unsync) {
            //clear the buffer
            Packet_Decoder_resetBuffers(d);
            //clear the checksum, just in case
            d->checksum = 0;
            if (c == packet_decoder_binary_header_1) {
                d->status = Sync;
            }
        } else if (d->status == Sync) {
            if (c == packet_decoder_binary_header_2) {
                d->status = Length;
            } else {
                d->status = Unsync;
            }
        } else if (d->status == Length) {
            d->lenght = (uint8_t) c;
            d->status = Payload;
        } else if (d->status == Payload) {
            if (d->lenght > 0) {
                d->checksum ^= c;
                *(d->bufferPtr) = c;
                d->bufferPtr++;
                d->lenght--;
            } else {
                d->checksum ^= c;
                d->status = Unsync;
                if (d->checksum == 0) return 1; //checksum good
                else return 0; //checksum error;
            }

        } else {
            d->status = Unsync;
        }
    }        //-----------------------------------------------------
    //ascii mode
    //-----------------------------------------------------
    else if (d->ascii == 1) {
        if (d->status == Unsync) {
            //clear the buffer
            Packet_Decoder_resetBuffers(d);
            if (c == packet_decoder_ascii_header) {

                d->status = Payload;
            }
        } else if (d->status == Payload) {
            //Sync Error, clean buffer and start oveer
            if (c == packet_decoder_ascii_header) {
                d->status = Unsync;
            }//Packet end, process it
            else if (c == packet_decoder_ascii_footer) {
                d->status = Unsync;
                return 1;
            }//in any other case, the char received is probably good!
            else {
                *(d->bufferPtr) = c;
                d->bufferPtr++;
            }
        }
    }
    return 0;
}
//==============================================================================
//==============================================================================


//PACKET STUFF
//==============================================================================
//GenericPacket Utilities
//==============================================================================

char* Packet_write(const struct Packet* p, char* buffer, int ascii) {
    //Binary mode
    char* buffPtr;
    if (!ascii) {
        buffer = writeUint8(packet_decoder_binary_header_1, buffer);
        buffer = writeUint8(packet_decoder_binary_header_2, buffer);
    }
    if (p->id == Dummy_Payload_ID) {
        buffPtr = buffer;
        buffer = Dummy_Payload_write(p, buffer, ascii);
    } else if (p->id == Dumb_Payload_ID) {
        buffPtr = buffer;
        buffer = Dumb_Payload_write(p, buffer, ascii);
    } else if (p->id == State_Payload_ID) {
        buffPtr = buffer;
        buffer = State_Payload_write(p, buffer, ascii);
    } else if (p->id == Speed_Payload_ID) {
        buffPtr = buffer;
        buffer = Speed_Payload_write(p, buffer, ascii);
    } else if (p->id == Init_Payload_ID) {
        buffPtr = buffer;
        buffer = Init_Payload_write(p, buffer, ascii);
    }else if (p->id == Heartbeat_Payload_ID) {
        buffPtr = buffer;
        buffer = Heartbeat_Payload_write(p, buffer, ascii);
    }
    //Binary mode, has to compute the checksum
    if (!ascii) {
        uint8_t checksum = Payload_computeChecksum(buffer, buffer - buffPtr);
        buffer = writeUint8(checksum, buffer);
    }
    return buffer;
}

//Note,buffer has to point to the ID, not to the length

char* Packet_parse(char* buffer, struct Packet* p, int ascii) {
    uint8_t id;
    if (!ascii) {
        buffer = readUint8(&id, buffer);
    } else {
        buffer = readUint8Ascii((char*) &id, buffer);
    }
    //IF - ELSE-IF for each existing packet
    p->id = id;
    if (id == Dummy_Payload_ID) {
        Dummy_Payload_read(p, buffer, ascii);
    } else if (id == Dumb_Payload_ID) {
        Dumb_Payload_read(p, buffer, ascii);
    } else if (id == State_Payload_ID) {
        State_Payload_read(p, buffer, ascii);
    } else if (id == Speed_Payload_ID) {
        Speed_Payload_read(p, buffer, ascii);
    } else if (id == Init_Payload_ID) {
        Init_Payload_read(p, buffer, ascii);
    }else if (id == Heartbeat_Payload_ID) {
        Heartbeat_Payload_read(p, buffer, ascii);
    }
    return buffer;
}

//calls the right execute for the packet, returns the id

int Packet_execute(struct Packet* p) {
    if (p->id == Dummy_Payload_ID) {
        Dummy_Payload_execute(p);
    } else if (p->id == Dumb_Payload_ID) {
        Dumb_Payload_execute(p);
    } else if (p->id == State_Payload_ID) {
        State_Payload_execute(p);
    } else if (p->id == Speed_Payload_ID) {
        Speed_Payload_execute(p);
    } else if (p->id == Init_Payload_ID) {
        Init_Payload_execute(p);
    }else if (p->id == Heartbeat_Payload_ID) {
        Heartbeat_Payload_execute(p);
    }
    return p->id;
}

uint8_t Payload_computeChecksum(char* buffer, uint8_t length) {
    uint8_t checksum = 0;
    uint8_t len = length - 1;
    while (len) {
        checksum ^= *(buffer - len);
        len--;
    }
    return checksum;
}






//Uart facilities
//==============================================================================
#ifdef __33FJ128MC802_H
//use this only on the micro side

void sendToUart(char* buffer, int length, int intraCharDelayUs) {
    int i = 0;
    while (i < length) {
        U1TXREG = buffer[i];
        i++;
        DelayN10us(intraCharDelayUs);
    }
}
#else

void sendToUart(int device, char* buffer, int length, int intraCharDelayUs) {
    int i = 0;
    while (i < length) {
        write(device, &buffer[i], 1);
        i++;
        usleep(intraCharDelayUs);
    }
}
#endif
//==============================================================================
//==============================================================================
//==============================================================================
//==============================================================================
