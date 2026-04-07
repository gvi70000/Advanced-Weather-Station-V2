#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>   /* ? add this line */

#define DEBUG_ENABLED 1
#if DEBUG_ENABLED
    #define DEBUG printf
#else
    #define DEBUG(...) 
#endif

#endif // DEBUG_H