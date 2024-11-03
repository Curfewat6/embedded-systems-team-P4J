#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwipopts.h"
#include <stdio.h>
#include <stdlib.h>
#include "hardware/rtc.h"
#include "time.h"
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define NTP_SERVER  "time.google.com"
#define NTP_PORT    (123)
#define NTP_DELTA (2208988800) // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_MSG_LEN (48)  // ignore Authenticator (optional)

typedef struct NTP_TIME_T {
    ip_addr_t ntp_ipaddr;
    struct udp_pcb *ntp_pcb;
    bool ntp_server_found;
    absolute_time_t ntp_update_time;
    int tcpip_link_state;
} NTP_TIME;
NTP_TIME net_time;

int print_rtc_time(){
    datetime_t rtc_time;
    if (rtc_get_datetime(&rtc_time)) { // Retrieve current RTC time
        printf("Current RTC Time: %02d/%02d/%04d %02d:%02d:%02d\n",
               rtc_time.day, rtc_time.month, rtc_time.year,
               rtc_time.hour, rtc_time.min, rtc_time.sec);
    }
    else {
        printf("Error reading RTC time\n");
    }
}

void ntp_recv_cb(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    printf("Receive NTP response\n");
    NTP_TIME* ntp_time = (NTP_TIME*)arg;
    uint8_t mode=pbuf_get_at(p,0)& 0x07;  // LI[2], VN[3], MODE[3], mode(0x04): server
    uint8_t stratum = pbuf_get_at(p,1);   // straum 
    uint8_t ts[4]={0};
    uint32_t sec_offset;
    if (port == NTP_PORT && ip_addr_cmp(&net_time.ntp_ipaddr, addr) && p->tot_len == NTP_MSG_LEN && mode == 0x04 && stratum != 0) { 
        pbuf_copy_partial(p, ts, sizeof(ts), 40);
        sec_offset = ((uint32_t)ts[0])<<24 | ((uint32_t)ts[1])<<16 | ((uint32_t)ts[2])<<8 | ((uint32_t)ts[3]);
        uint32_t temp = sec_offset - NTP_DELTA+8*60*60; //UTC+8
        time_t utc_sec_offset = temp;
        struct tm *utc = gmtime(&utc_sec_offset);
        datetime_t rtc_time;
 
        rtc_time.year=utc->tm_year+1900;
        rtc_time.month= utc->tm_mon+1;
        rtc_time.day = utc->tm_mday;
        rtc_time.hour = utc->tm_hour;
        rtc_time.min = utc->tm_min;
        rtc_time.sec = utc->tm_sec;
        rtc_time.dotw = utc->tm_wday;
        if (!rtc_set_datetime(&rtc_time)) printf("set rtc error\n");
 
        printf("got ntp response: %02d/%02d/%04d %02d:%02d:%02d\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
               utc->tm_hour, utc->tm_min, utc->tm_sec);

        if (!rtc_set_datetime(&rtc_time)) {
            printf("Set RTC error\n");
        }

        ntp_time->ntp_update_time = make_timeout_time_ms(21600000); //6*60*60*1000        
    }
    pbuf_free(p);
}
 
void ntp_init_data() {
    net_time.ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    net_time.ntp_server_found=false;
    net_time.tcpip_link_state = CYW43_LINK_DOWN;
    if (!net_time.ntp_pcb) {
        printf("alloc udp_new error\n");
        return;
    }
    udp_recv(net_time.ntp_pcb, ntp_recv_cb, &net_time);
}
 
void get_ntp_time() {
    cyw43_arch_lwip_begin();
    printf("Inside get_ntp_time()\n");
    struct pbuf *pb = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *) pb->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;   // 0x00 011 011 (LI:00, VN:3(version), MODE:3 (client))
    printf("Sending UDP packet\n");
    err_t err = udp_sendto(net_time.ntp_pcb, pb, &net_time.ntp_ipaddr, NTP_PORT);
    if (err != ERR_OK){
        printf("Error sending UDP packet: %d\n", err);
    }
    else {
        printf("UDP packet sent\n");
        for (int i = 0; i < 5; i++) {
            sleep_ms(1000);  // Increase wait time if needed
        }
    }
    pbuf_free(pb);
    cyw43_arch_lwip_end();
}
 
void dns_cb(const char *name, const ip_addr_t *ipaddr, void *arg) {
    NTP_TIME* ntime = (NTP_TIME*)(arg);
    ntime->ntp_ipaddr=*ipaddr;
    printf("ntp server:%s\n", ipaddr_ntoa(&ntime->ntp_ipaddr));
    ntime->ntp_server_found = true;
}

bool resolve_ntp_server() {
    printf("Resolving NTP server...\n");
    int dns_ret;
    absolute_time_t timeout = make_timeout_time_ms(20000);

    while (!net_time.ntp_server_found && absolute_time_diff_us(get_absolute_time(), timeout) > 0) {
        dns_ret = dns_gethostbyname(NTP_SERVER, &net_time.ntp_ipaddr, dns_cb, &net_time);
        if (dns_ret == ERR_OK) {
            net_time.ntp_server_found = true;
            break;
        }
        sleep_ms(1000);
    }

    if (!net_time.ntp_server_found) {
        printf("NTP server not found!\n");
        return false;
    }

    return true;
}