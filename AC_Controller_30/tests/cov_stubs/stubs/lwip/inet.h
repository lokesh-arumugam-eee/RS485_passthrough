#pragma once
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdio.h>

/* ESP-IDF/lwip defines inet_ntoa as a macro that takes the raw uint32_t address
 * field, not a struct in_addr.  Override the POSIX function so that
 * firmware calls like inet_ntoa(poll_ip.addr) compile and run correctly. */
#undef inet_ntoa
static inline char* _lwip_stub_inet_ntoa(uint32_t a) {
    static char _lbuf[16];
    snprintf(_lbuf, sizeof(_lbuf), "%u.%u.%u.%u",
        (unsigned)(a & 0xff), (unsigned)((a >> 8) & 0xff),
        (unsigned)((a >> 16) & 0xff), (unsigned)((a >> 24) & 0xff));
    return _lbuf;
}
#define inet_ntoa(addr) _lwip_stub_inet_ntoa((uint32_t)(addr))
