// =====================================
// File: lq_tx.c   (Node A – transmitter)
// =====================================

#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include <string.h>
#include <stdio.h>

#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02

#define BEACON_PERIOD   (2*CLOCK_SECOND)   // 2 s
#define RSSI_THRESHOLD  (-70)              // dBm
#define GOOD_COUNT_REQ  3                  // need 3 consecutive good beacons

static uint8_t beacon_byte = PKT_BEACON;
static struct etimer beacon_timer;
static uint8_t good_count = 0;
static linkaddr_t peer_addr;
static uint8_t peer_set = 0;
static uint8_t request_sent = 0;

PROCESS(lq_tx_proc, "Link‑quality TX (Node A)");
AUTOSTART_PROCESSES(&lq_tx_proc);

/* ---------------- input callback ---------------- */
static void input_cb(const void *data, uint16_t len,
                     const linkaddr_t *src, const linkaddr_t *dest)
{
  if(len==1 && ((const uint8_t*)data)[0]==PKT_BEACON) {
     signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);
     printf("%lu RX_BEACON from %02x:%02x  RSSI=%d\n", clock_seconds(), src->u8[0], src->u8[1], rssi);

     if(rssi >= RSSI_THRESHOLD) {
        if(!peer_set) {
          linkaddr_copy(&peer_addr, src);
          peer_set = 1;
          good_count = 1;
        } else if(linkaddr_cmp(src,&peer_addr)) {
          good_count++;
        }
     } else {
        if(peer_set && linkaddr_cmp(src,&peer_addr)) good_count=0;
     }

     if(peer_set && good_count >= GOOD_COUNT_REQ && !request_sent) {
        printf("%lu DETECT %u\n", clock_seconds(), peer_addr.u8[7]);
        uint8_t req = PKT_REQUEST;
        nullnet_buf = &req; nullnet_len = 1;
        NETSTACK_NETWORK.output(&peer_addr);
        request_sent = 1;
     }
  }
}

/* ---------------- main process ---------------- */
PROCESS_THREAD(lq_tx_proc, ev, data)
{
  PROCESS_BEGIN();
  nullnet_set_input_callback(input_cb);

  etimer_set(&beacon_timer, BEACON_PERIOD);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&beacon_timer));
    nullnet_buf = &beacon_byte; nullnet_len = 1;
    NETSTACK_NETWORK.output(NULL);
    printf("%lu BEACON_SENT\n", clock_seconds());
    etimer_reset(&beacon_timer);
  }
  PROCESS_END();
}
