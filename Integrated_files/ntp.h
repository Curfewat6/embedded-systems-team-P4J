// ntp_client.h

#ifndef NTP_CLIENT_H
#define NTP_CLIENT_H

#include "pico/stdlib.h"
#include "lwip/ip_addr.h"
#include "lwip/udp.h"

// Constants
#define NTP_SERVER "time.google.com"
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_MSG_LEN 48       // ignore Authenticator (optional)

// Structure for storing NTP time info
typedef struct NTP_TIME_T {
    ip_addr_t ntp_ipaddr;
    struct udp_pcb *ntp_pcb;
    bool ntp_server_found;
    absolute_time_t ntp_update_time;
} NTP_TIME;

// Function declarations
int print_rtc_time();
void ntp_init_data();
bool resolve_ntp_server();
void get_ntp_time();
void dns_cb(const char *name, const ip_addr_t *ipaddr, void *arg);
void ntp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

#endif // NTP_CLIENT_H
