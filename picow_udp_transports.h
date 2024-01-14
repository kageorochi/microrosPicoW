#ifndef MICRO_ROS_PICOSDK
#define MICRO_ROS_PICOSDK

#include <stdio.h>
#include <stdint.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "pico/cyw43_arch.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#define ROS_AGENT_UDP_PORT  (8888)
#define ROS_AGENT_IP_ADDR   "192.168.24.17"    // You need modify IP

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
    struct udp_pcb* pcb;
    ip_addr_t ipaddr;
    uint16_t port;
} ST_PICOW_TRANSPORT_PARAMS;

static ST_PICOW_TRANSPORT_PARAMS picow_params = { 0 };

#if 0
bool picow_udp_transport_open(struct uxrCustomTransport * transport);
bool picow_udp_transport_close(struct uxrCustomTransport * transport);
size_t picow_udp_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t picow_udp_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);
#endif

#ifdef __cplusplus
}
#endif

#endif //MICRO_ROS_PICOSDK
