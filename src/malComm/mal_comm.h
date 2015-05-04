/*
 * File:   MalComm.h
 * Author: malcom
 *
 * Created on February 17, 2015, 11:27 PM
 */

#ifndef MAL_COMM
#define MAL_COMM

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifdef TOPORAGNAROK
#include "serialStuff.h"
#include <unistd.h>
#else
#include <p33FJ128MC802.h>
#include "generic_utils.h"
#endif
#include "mal_primitives.h"

#define BUFFER_SIZE 255


//PACKETS
//==============================================================================
//==============================================================================

//CONFIGURATION PACKET
//==============================================================================
//useful to get/set eeprom parameters
extern uint8_t Configuration_Payload_ID;

struct Configuration_Payload {
    uint8_t address;
    int16_t intValue;
    float floatValue;
};


//SLIM STATE PACKET
//==============================================================================
extern uint8_t State_Payload_ID;

struct State_Payload {
    uint16_t leftEncoder;
    uint16_t rightEncoder;
};


//INIT PACKET
//==============================================================================
//used to start/reset the roboot
extern uint8_t Init_Payload_ID;

struct Init_Payload {
    uint8_t init;
};


//HEARBEAT PACKET
//==============================================================================
//DESCRIPTION HERE
extern uint8_t Heartbeat_Payload_ID;
struct Heartbeat_Payload {
    uint16_t beat;
};


//SPEED PACKET
//==============================================================================
//Speed packet is used by the client to request a specific speed on the motors
//The speed is expressed in the form of encoder ticks
extern uint8_t Speed_Payload_ID;

struct Speed_Payload {
    int16_t leftTick;
    int16_t rightTick;
};

//DUMB PACKET
//==============================================================================
extern uint8_t Dumb_Payload_ID;

struct Dumb_Payload {
    uint8_t stupid;
};
//==============================================================================


//DUMMY PACKET
//==============================================================================
extern uint8_t Dummy_Payload_ID;

struct Dummy_Payload {
    uint8_t field_1;
    uint16_t field_2;
    int8_t field_3;
    char field_4;
    float field_5;
    unsigned char field_6;
    uint32_t field_7;
    int32_t field_8;
};
//==============================================================================

struct Packet {
    uint8_t id;
    uint8_t lenght;
    uint32_t seq;

    union {
        struct Dummy_Payload dummy;
        struct Dumb_Payload dumb;
        struct State_Payload state;
        struct Init_Payload init;
        struct Speed_Payload speed;
        struct Heartbeat_Payload heartbeat;
    };
};

struct Packet_Decoder {
    int ascii;
    int status;
    char buffer[BUFFER_SIZE];
    char* buffer_start;
    char* bufferPtr;
    uint8_t lenght;
    uint8_t checksum;
};
//==============================================================================
//==============================================================================



//Externs and Union
//==============================================================================

enum buffer_status {
    Unsync = 0x0, Sync = 0x1, Length = 0x2, Payload = 0x3
};
extern char packet_decoder_binary_header_1;
extern char packet_decoder_binary_header_2;
extern char packet_decoder_ascii_header;
extern char packet_decoder_ascii_footer;
//==============================================================================
//==============================================================================


//Packets Stuff
//==============================================================================
//WritePacket:
//gets a const packet to create a char array ready to be sent
char* Packet_write(const struct Packet* p, char* buffer, int ascii);
//ParsePacket:
char* Packet_parse(char* buffer, struct Packet* p, int ascii);
//Execute
int Packet_execute(struct Packet* p);
//==============================================================================


//[WR]DUMMY PACKET
char* Dummy_Payload_write(const struct Packet* p, char* buffer, int ascii);
char* Dummy_Payload_read(struct Packet* p, char* buffer, int ascii);
void Dummy_Payload_execute(struct Packet* p);
//==============================================================================
//[WR]DUMB PACKET
char* Dumb_Payload_write(const struct Packet* p, char* buffer, int ascii);
char* Dumb_Payload_read(struct Packet* p, char* buffer, int ascii);
void Dumb_Payload_execute(struct Packet* p);
//==============================================================================
//[WR]SLIMSTATE PACKET
char* State_Payload_write(const struct Packet* p, char* buffer, int ascii);
char* State_Payload_read(struct Packet* p, char* buffer, int ascii);
void State_Payload_execute(struct Packet* p);
//==============================================================================
//[WR]INIT PACKET
char* Init_Payload_write(const struct Packet* p, char* buffer, int ascii);
char* Init_Payload_read(struct Packet* p, char* buffer, int ascii);
void Init_Payload_execute(struct Packet* p);
//==============================================================================
//[WR]SPEED PACKET
char* Speed_Payload_write(const struct Packet* p, char* buffer, int ascii);
char* Speed_Payload_read(struct Packet* p, char* buffer, int ascii);
void Speed_Payload_execute(struct Packet* p);
//==============================================================================
//[WR]HEARTBEAT PACKET
char* Heartbeat_Payload_write(const struct Packet* p, char* buffer, int ascii);
char* Heartbeat_Payload_read(struct Packet* p, char* buffer, int ascii);
void Heartbeat_Payload_execute(struct Packet* p);
//==============================================================================

//DECODER STUFF
//Utility functions to clear and prepare the encoder/decoder stuff.
//Call it before use it!
//==============================================================================
void Packet_Decoder_init(struct Packet_Decoder* dm, int ascii);
void Packet_Decoder_resetBuffers(struct Packet_Decoder* d);
int Packet_Decoder_putChar(struct Packet_Decoder* d, char c);
void Dummy_Payload_execute(struct Packet* p);
//==============================================================================
//==============================================================================




//GenericStuff
//==============================================================================
void initConsts();
//Checksum
uint8_t Payload_computeChecksum(char* buffer, uint8_t length);
//==============================================================================
//==============================================================================


#ifdef __33FJ128MC802_H
//Generic send to uart
void sendToUart(char* buffer, int length, int intraCharDelayUs);
#else
void sendToUart(int device, char* buffer, int length, int intraCharDelayUs);
#endif

#endif
