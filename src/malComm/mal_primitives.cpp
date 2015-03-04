#include "mal_primitives.h"

//==============================================================================
//Conversion primitives:BINARY

char* writeChar(char value, char* buffer) {
    *(char*) buffer = value;
    buffer++;
    return buffer;
}

char* writeUint8(uint8_t value, char* buffer) {
    *(uint8_t*) buffer = value;
    buffer++;
    return buffer;
}

char* writeUint16(uint16_t value, char* buffer) {
    char* valuePTR = (char*) &value;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    return buffer;
}

char* writeUint32(uint32_t value, char* buffer) {
    char* valuePTR = (char*) &value;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    return buffer;
}

char* writeFloat(float value, char* buffer) {
    char* valuePTR = (char*) &value;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    valuePTR++;
    *(char*) buffer = *valuePTR;
    buffer++;
    return buffer;
}

char* readUint8(uint8_t* dest, char* buffer) {
    *dest = *(uint8_t*) buffer;
    return buffer + 1;
}

char* readUint16(uint16_t* dest, char* buffer) {
    char* valuePTR = (char*) dest;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    return buffer;
}

char* readUint32(uint32_t* dest, char* buffer) {
    char* valuePTR = (char*) dest;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    return buffer;
}

char* readFloat(float* dest, char* buffer) {
    char* valuePTR = (char*) dest;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    valuePTR++;
    *(char*) valuePTR = *buffer;
    buffer++;
    return buffer;
}


//==============================================================================
//Conversion primitives:ASCII

char* writeHeaderAscii(char* buffer) {
    return buffer + sprintf(buffer, "%c ", packet_decoder_ascii_header);
}

char* writeFooterAscii(char* buffer) {
    return buffer + sprintf(buffer, "%c ", packet_decoder_ascii_footer);
}

char* writeCharAscii(char value, char* buffer) {
    return buffer + sprintf(buffer, "%u ", value);

}

char* writeUint8Ascii(uint8_t value, char* buffer) {
    return buffer + sprintf(buffer, "%u ", value);

}

char* writeUint16Ascii(uint16_t value, char* buffer) {
    return buffer + sprintf(buffer, "%u ", value);

}

char* writeUint32Ascii(uint32_t value, char* buffer) {
    return buffer + sprintf(buffer, "%lu ", value);

}

char* writeFloatAscii(float value, char* buffer) {
    return buffer + sprintf(buffer, "%f ", (double) value);

}

char* readUint8Ascii(char* dest, char* buffer) {
    *(uint8_t*) dest = (uint8_t) strtoul(buffer, &buffer, 10);
    return buffer;
}

char* readUint16Ascii(char* dest, char* buffer) {
    *(uint16_t*) dest = strtoul(buffer, &buffer, 10);
    return buffer;
}

char* readUint32Ascii(char* dest, char* buffer) {
    *(uint32_t*) dest = strtoul(buffer, &buffer, 10);
    return buffer;
}

char* readFloatAscii(char* dest, char* buffer) {
    *(float*) dest = strtoul(buffer, &buffer, 10);
    return buffer;
}