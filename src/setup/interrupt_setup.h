#ifndef NVIC_H
#define NVIC_H

// Include target-specific interrupt setup
#if defined(TARGET_CLN17V3)
    #include "target/cln17_v3/interrupt_setup.h"
#elif defined(TARGET_RADIX)
    #include "target/radix/interrupt_setup.h"
#else
    #error "No target defined! Define TARGET_RADIX or TARGET_CLN17V3"
#endif

#endif  // NVIC_H