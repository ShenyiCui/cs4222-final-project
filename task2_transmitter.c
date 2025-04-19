// ================================
// File: beacon_tx.c   (transmitter)
// ================================

#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include <stdio.h>

#define PKT_BEACON 0x01
#define BEACON_PERIOD (2*CLOCK_SECOND)   // 2‑second interval

PROCESS(beacon_tx_proc, "Basic beacon transmitter");
AUTOSTART_PROCESSES(&beacon_tx_proc);

PROCESS_THREAD(beacon_tx_proc, ev, data)
{
  static struct etimer timer;
  static uint8_t beacon = PKT_BEACON;
  PROCESS_BEGIN();

  nullnet_buf = &beacon;
  nullnet_len = 1;

  etimer_set(&timer, BEACON_PERIOD);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));

    NETSTACK_NETWORK.output(NULL);  // broadcast
    printf("%lu BEACON_SENT\n", clock_seconds());

    etimer_reset(&timer);
  }
  PROCESS_END();
}