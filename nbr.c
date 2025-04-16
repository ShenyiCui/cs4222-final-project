/*
* CS4222/5422: Assignment 3b
* Perform neighbour discovery with a two–phase mechanism for deterministic
* two-way discovery within 10 seconds while conserving power.
*
* In this approach, nodes start in a low–duty cycle (phase 0).
* When a node receives a beacon (from a different node) it switches to
* aggressive mode (phase 1) and records the time.
* While in aggressive mode, the node transmits with minimal sleep (i.e., high duty cycle)
* until it receives an aggressive beacon from its partner. Then, it transitions
* to phase 2 (discovery complete) and stops scanning.
* If the 10-second aggressive phase elapses without two-way discovery, the node
* reverts back to low duty cycle (phase 0) and continues discovery.
*/

#include "contiki.h"
#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "lib/random.h"
#include "net/linkaddr.h"
#include <string.h>
#include <stdio.h> 
#include "node-id.h"

// Configures the wake-up timer for neighbour discovery 
#define WAKE_TIME RTIMER_SECOND/10    // 10 Hz, 0.1 s
#define SLEEP_CYCLE  4                // Default value (used when not in aggressive mode)
#define SLEEP_SLOT RTIMER_SECOND/10   // 0.1 s

// Use broadcast address for neighbour discovery:
linkaddr_t dest_addr;
#define NUM_SEND 2

// In low mode we choose a somewhat conservative (power–saving) sleep count.
#define LOW_SLEEP_COUNT  (2 * SLEEP_CYCLE)  // Adjust as needed for power saving

/*---------------------------------------------------------------------------*/
// Data packet now carries a phase field:
// phase = 0: low duty cycle; phase = 1: aggressive mode; phase = 2: discovery complete (sleep)
typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
  uint8_t phase;
} data_packet_struct;

/*---------------------------------------------------------------------------*/
// Global timing and protothread variables
static struct rtimer rt __attribute__((unused));
static struct pt pt;
static data_packet_struct data_packet;
unsigned long curr_timestamp;

// Global variables for managing aggressive mode
// 0: phase 0 (low duty cycle), 1: phase 1 (aggressive mode), 2: phase 2 (discovery complete)
static uint8_t mode = 0;
static unsigned long aggressive_start_time = 0;

/*---------------------------------------------------------------------------*/
// Process declaration is mandatory.
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");

/*---------------------------------------------------------------------------*/
// Receive callback: process incoming discovery packets.
void receive_packet_callback(const void *data, uint16_t len, 
                             const linkaddr_t *src, const linkaddr_t *dest) {
  printf("Received packet from %d.%d to %d.%d\n",
         src->u8[0], src->u8[1], dest->u8[0], dest->u8[1]);
  if(len == sizeof(data_packet_struct)) {
    static data_packet_struct received_packet;
    memcpy(&received_packet, data, len);

    printf("Received neighbour discovery packet %lu with rssi %d from %ld, phase %d\n",
           received_packet.seq,
           (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI),
           received_packet.src_id,
           received_packet.phase);

    // If the packet is from a different node:
    if(received_packet.src_id != data_packet.src_id) {
      if(mode == 0) {
        // Not yet in aggressive mode: switch to phase 1
        mode = 1;
        aggressive_start_time = clock_time();
        printf("Switching to aggressive mode at %lu ticks\n", aggressive_start_time);
      } else if(mode == 1 && received_packet.phase == 1) {
        // Both nodes are in aggressive mode -> two-way discovery complete.
        mode = 2;
        printf("Two-way discovery complete at %lu ticks. Entering sleep mode.\n", clock_time());
      }
    }
  }
}

/*---------------------------------------------------------------------------*/
// Sender scheduler: transmits discovery beacons.
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int sleep_count = 0;
  unsigned long current;

  PT_BEGIN(&pt);

  curr_timestamp = clock_time();
  printf("Start clock %lu ticks, timestamp %3lu.%03lu\n",
         curr_timestamp,
         curr_timestamp / CLOCK_SECOND,
         ((curr_timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND);

  while(1) {
    // If discovery is complete (phase 2), stop transmissions.
    if(mode == 2) {
      printf("Discovery complete, stopping transmissions and entering sleep mode.\n");
      NETSTACK_RADIO.off();
      PT_EXIT(&pt);
    }

    // Turn the radio on before transmitting.
    NETSTACK_RADIO.on();

    // Set the data packet's phase field to current mode.
    data_packet.phase = mode;

    // Transmit the discovery beacons.
    for(i = 0; i < NUM_SEND; i++) {
      nullnet_buf = (uint8_t *)&data_packet;
      nullnet_len = sizeof(data_packet);

      data_packet.seq++;
      curr_timestamp = clock_time();
      data_packet.timestamp = curr_timestamp;

      printf("Send seq# %lu  @ %8lu ticks   %3lu.%03lu, phase %d\n",
             data_packet.seq,
             curr_timestamp,
             curr_timestamp / CLOCK_SECOND,
             ((curr_timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND,
             data_packet.phase);

      NETSTACK_NETWORK.output(&dest_addr);

      if(i != (NUM_SEND - 1)) {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1,
                   (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

    // Turn off the radio to save power.
    NETSTACK_RADIO.off();

    current = clock_time();

    // Determine sleep interval based on the current mode.
    if(mode == 1) {
      // In aggressive mode, use minimal sleep (1 slot) to maximize packet frequency.
      // Also, if the aggressive phase has lasted beyond 10 seconds without completing,
      // revert back to low duty cycle (phase 0).
      if((current - aggressive_start_time) < (10 * CLOCK_SECOND)) {
        sleep_count = 1;
      } else {
        // Aggressive phase expired without two-way discovery.
        mode = 0;
        sleep_count = LOW_SLEEP_COUNT;
        printf("Aggressive phase timed out. Reverting to low duty cycle mode.\n");
      }
    } else {  // mode == 0: low duty cycle mode.
      sleep_count = LOW_SLEEP_COUNT;
    }

    printf("Sleep for %d slots (mode %d)\n", sleep_count, mode);
    for(i = 0; i < sleep_count; i++) {
      rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1,
                 (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }
  }

  PT_END(&pt);
}

/*---------------------------------------------------------------------------*/
// Main process to initialize neighbour discovery.
// Apply __attribute__((used)) as required.
__attribute__((used))
PROCESS_THREAD(nbr_discovery_process, ev, data) {
  PROCESS_BEGIN();

  // Initialize our data packet.
  data_packet.src_id = node_id;
  data_packet.seq = 0;
  data_packet.phase = 0;  // Start in low duty cycle mode (phase 0).

  nullnet_set_input_callback(receive_packet_callback);
  linkaddr_copy(&dest_addr, &linkaddr_null);

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending packet of size %d Bytes\n",
         node_id, (int)sizeof(data_packet_struct));

  // Start the sender shortly after boot.
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1,
             (rtimer_callback_t)sender_scheduler, NULL);

  PROCESS_END();
}

// Autostart our process.
AUTOSTART_PROCESSES(&nbr_discovery_process);
