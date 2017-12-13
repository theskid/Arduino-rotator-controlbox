/*
**  Serial print helpers with debug and printf support
*/
#pragma once

#define SPFBUFSIZE 1024                                                 // 1KB single message buffer

// Serial.print wrapper enabling printf
/*void SerialPrintf(const char* format, ...) {
    char buffer[SPFBUFSIZE];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, SPFBUFSIZE, format, args);
    va_end(args);
    Serial.print(buffer);
}*/

// Support for flash-stored strings – F(string_literal)
void SerialPrintf(const __FlashStringHelper *format, ... ) {
    char buffer[SPFBUFSIZE];
    va_list args;
    va_start(args, format);
    #ifdef __AVR__
        vsnprintf_P(buffer, sizeof(buffer), (const char*)format, args); // Progmem for AVR
    #else
        vsnprintf(buffer, sizeof(buffer), (const char*)format, args);   // For the rest of the world
    #endif
    va_end(args);
    Serial.print(buffer);
}

#undef SPFBUFSIZE

#ifdef DEBUG
    #define DebugPrintf(x, ...) SerialPrintf(F(x), __VA_ARGS__)
    #define DebugPrint(x) Serial.print(F(x))
#else
    #define DebugPrintf(...) (void(0))
    #define DebugPrint(...) (void(0))
#endif