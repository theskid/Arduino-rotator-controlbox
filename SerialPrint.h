/*
**  Serial print helpers with debug and printf support
*/
#pragma once

void SerialPrintf(const char* format, ...);

#ifdef DEBUG
    #define DebugPrintf(x, ...) SerialPrintf(x, __VA_ARGS__)
    #define DebugPrint(x) Serial.print(x)
#else
    #define DebugPrintf(...) (void(0))
    #define DebugPrint(...) (void(0))
#endif
