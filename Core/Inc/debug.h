#ifndef DEBUG_H
#define DEBUG_H

// Enable or disable debug printing
#define DEBUG_ENABLED 1  

#if DEBUG_ENABLED
    #define DEBUG printf
#else
    #define DEBUG(...) // No-op when debugging is disabled
#endif

#endif // DEBUG_H