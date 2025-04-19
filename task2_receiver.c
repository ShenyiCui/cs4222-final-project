// ================================
// File: beacon_rx.c   (receiver)
// ================================

#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include <stdio.h>

#define PKT_BEACON 0x01

PROCESS(beacon_rx_proc, "Basic beacon receiver");
AUTOSTART_PROCESSES(&beacon_rx_proc);

static void input_cb(const void *data, uint16_t len,
                     const linkaddr_t *src, const linkaddr_t *dest)
{
  if(len == 1 && ((const uint8_t *)data)[0] == PKT_BEACON) {
    int rssi = (int)packetbuf_attr(PACKETBUF_ATTR_RSSI);
    printf("%lu BEACON_RX from %02x:%02x  RSSI: %d dBm\n",
           clock_seconds(), src->u8[0], src->u8[1], rssi);
  }
}

PROCESS_THREAD(beacon_rx_proc, ev, data)
{
  PROCESS_BEGIN();
  nullnet_set_input_callback(input_cb);
  PROCESS_END();
}
