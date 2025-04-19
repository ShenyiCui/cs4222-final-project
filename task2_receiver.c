
// =====================================
// File: lq_rx.c   (Node B – receiver)
// =====================================

#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include <stdio.h>

#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02

#define BEACON_PERIOD (2*CLOCK_SECOND)

static uint8_t beacon_byte = PKT_BEACON;
static struct etimer beacon_timer;

PROCESS(lq_rx_proc, "Link‑quality RX (Node B)");
AUTOSTART_PROCESSES(&lq_rx_proc);

static void send_beacon(void)
{
  nullnet_buf = &beacon_byte; nullnet_len = 1;
  NETSTACK_NETWORK.output(NULL);
  printf("%lu BEACON_SENT\n", clock_seconds());
}

static void input_cb(const void *data, uint16_t len,
                     const linkaddr_t *src, const linkaddr_t *dest)
{
  if(len==1) {
    uint8_t type = ((const uint8_t*)data)[0];
    if(type==PKT_REQUEST) {
       signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
       printf("%lu DETECT %u RSSI: %d\n", clock_seconds(), src->u8[7], rssi);
    } else if(type==PKT_BEACON) {
       signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
       printf("%lu RX_BEACON from %02x:%02x RSSI=%d\n", clock_seconds(), src->u8[0], src->u8[1], rssi);
    }
  }
}

PROCESS_THREAD(lq_rx_proc, ev, data)
{
  PROCESS_BEGIN();
  nullnet_set_input_callback(input_cb);
  etimer_set(&beacon_timer, BEACON_PERIOD);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&beacon_timer));
    send_beacon();
    etimer_reset(&beacon_timer);
  }
  PROCESS_END();
}
