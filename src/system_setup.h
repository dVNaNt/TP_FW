#ifndef SYSTEM_SETUP_H
#define SYSTEM_SETUP_H

// Include target-specific system setup based on build configuration
#if defined(TARGET_CLN17V3)
    #include "target/cln17_v3/system_setup.h"
#elif defined(TARGET_RADIX)
    #include "target/radix/system_setup.h"
#else
    #error "No target defined! Define TARGET_RADIX or TARGET_CLN17V3"
#endif

#endif