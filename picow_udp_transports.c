#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include <picow_udp_transports.h>

uint8_t trans_recv_buff[512] = { 0 };
uint16_t trans_recv_len = 0;

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}

static void callback_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) arg;

    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    if (params) {
        // Check the result
        printf("callback_recv: ip[%s]port[%d]\n", ipaddr_ntoa(addr), port);
        if (ip_addr_cmp(addr, &params->ipaddr)) {
            if (trans_recv_len > 0) {
                printf("callback_recv: maybee data loss.. trans_recv_len(%d)\n", trans_recv_len);
            }
            trans_recv_len = pbuf_copy_partial(p, trans_recv_buff, sizeof(trans_recv_buff), 0);
            printf("callback_recv: trans_recv_len(%d)\n", trans_recv_len);
        }
        else {
            printf("callback_recv: invalid micro-ROS Agent response\n");
        }
    }
    pbuf_free(p);
    cyw43_arch_lwip_end();

    for (int i = 0; i < trans_recv_len; i++) {
        printf("%02x ", trans_recv_buff[i]);
    }
    printf("\n");

}

bool picow_udp_transport_open(struct uxrCustomTransport * transport)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;

    if (params) {
        // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
        // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
        // these calls are a no-op and can be omitted, but it is a good practice to use them in
        // case you switch the cyw43_arch type later.
        cyw43_arch_lwip_begin();
        params->pcb = udp_new();
        ipaddr_aton(ROS_AGENT_IP_ADDR, &(params->ipaddr));
        params->port = ROS_AGENT_UDP_PORT;

        udp_recv(params->pcb, callback_recv, params);
        cyw43_arch_lwip_end();

        printf("picow_udp_transport_open: SUCCESS\n");
        return true;
    }
    else {
        printf("picow_udp_transport_open: FAILURE\n");
        return false;
    }
}

bool picow_udp_transport_close(struct uxrCustomTransport * transport)
{
    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;

    if (params) {
        // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
        // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
        // these calls are a no-op and can be omitted, but it is a good practice to use them in
        // case you switch the cyw43_arch type later.
        cyw43_arch_lwip_begin();
        udp_remove(params->pcb);
        cyw43_arch_lwip_end();

        printf("picow_udp_transport_close: \n");
    }
    return true;
}

size_t picow_udp_transport_write(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, uint8_t *errcode)
{
    size_t trans_len = 0;
    *errcode = 1;   // failure

    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;
    if (params) {
        // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
        // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
        // these calls are a no-op and can be omitted, but it is a good practice to use them in
        // case you switch the cyw43_arch type later.
        cyw43_arch_lwip_begin();
        struct pbuf* p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
        if (p) {
            uint8_t* req = (uint8_t*) p->payload;
            memcpy(req, buf, len);
            err_t error = udp_sendto(params->pcb, p, &(params->ipaddr), params->port);
            if (error == ERR_OK) {
                *errcode = 0;   // success
                trans_len = len;
            }
            pbuf_free(p);
        }
        cyw43_arch_lwip_end();
    }

    printf("picow_udp_transport_write: len(%d) trans_len(%d)\n", len, trans_len);
    for (int i = 0; i < trans_len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");

    return trans_len;
}

size_t picow_udp_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
{
    size_t recv_len = 0;
    *errcode = 1;   // failure

#if PICO_CYW43_ARCH_POLL
    // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
    // main loop (not from a timer) to check for WiFi driver or lwIP work that needs to be done.
    cyw43_arch_poll();
#endif

    ST_PICOW_TRANSPORT_PARAMS* params = (ST_PICOW_TRANSPORT_PARAMS*) transport->args;
    if (params && (trans_recv_len > 0)) {
        // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
        // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
        // these calls are a no-op and can be omitted, but it is a good practice to use them in
        // case you switch the cyw43_arch type later.
        cyw43_arch_lwip_begin();
        if (trans_recv_len >= len) {
            recv_len = len;
            *errcode = 0;   // success
        }
        else {
            recv_len = trans_recv_len;
        }
        memcpy(buf, trans_recv_buff, recv_len);
        if (trans_recv_len > len) {
            printf("picow_udp_transport_read: maybee data loss.. trans_recv_len(%d) len(%d)\n", trans_recv_len, len);
        }
        trans_recv_len = 0;

        cyw43_arch_lwip_end();
    }

#if 0
    printf("picow_udp_transport_read: timeout(%d) len(%d) recv_len(%d)\n", timeout, len, recv_len);
    for (int i = 0; i < recv_len; i++) {
        printf("%02x ", buf[i]);
    }
    printf("\n");
#endif

    return recv_len;
}
