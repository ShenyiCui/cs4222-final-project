/*
* CS4222/5422: Assignment 3b
* Perform neighbour discovery
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

// Identification information of the node

// Configures the wake-up timer for neighbour discovery 
#define WAKE_TIME RTIMER_SECOND/10    // 10 HZ, 0.1s
#define SLEEP_CYCLE  9        	      // 0 for never sleep
#define SLEEP_SLOT RTIMER_SECOND/10   // sleep slot should not be too large to prevent overflow

// For neighbour discovery, we would like to send message to everyone. We use Broadcast address:
linkaddr_t dest_addr;

#define NUM_SEND 2

/*---------------------------------------------------------------------------*/
typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
} data_packet_struct;

/*---------------------------------------------------------------------------*/
// duty cycle = WAKE_TIME / (WAKE_TIME + SLEEP_SLOT * SLEEP_CYCLE)
/*---------------------------------------------------------------------------*/

// sender timer implemented using rtimer
static struct rtimer rt;

// Protothread variable
static struct pt pt;

// Structure holding the data to be transmitted
static data_packet_struct data_packet;

// Current time stamp of the node
unsigned long curr_timestamp;

// NEW: Variable to record the discovery start time
static unsigned long discovery_start_time;

// Starts the main contiki neighbour discovery process
PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");
AUTOSTART_PROCESSES(&nbr_discovery_process);

// Function called after reception of a packet
void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if(len == sizeof(data_packet)) {
    static data_packet_struct received_packet_data;
    memcpy(&received_packet_data, data, len);
    printf("Received neighbour discovery packet %lu with rssi %d from %ld", 
           received_packet_data.seq, (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI), received_packet_data.src_id);
    printf("\n");
  }
}

// Scheduler function for the sender of neighbour discovery packets
char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
  
  PT_BEGIN(&pt);

  curr_timestamp = clock_time();
  printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, 
         ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

  while(1) {
    // radio on
    NETSTACK_RADIO.on();

    // send NUM_SEND number of neighbour discovery beacon packets
    for(i = 0; i < NUM_SEND; i++) {
      nullnet_buf = (uint8_t *)&data_packet;
      nullnet_len = sizeof(data_packet);
      
      data_packet.seq++;
      curr_timestamp = clock_time();
      data_packet.timestamp = curr_timestamp;

      printf("Send seq# %lu  @ %8lu ticks   %3lu.%03lu\n", data_packet.seq, curr_timestamp, 
             curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND)*1000) / CLOCK_SECOND);

      NETSTACK_NETWORK.output(&dest_addr);
      
      if(i != (NUM_SEND - 1)){
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }

    // sleep for a number of slots
    if(SLEEP_CYCLE != 0){
      NETSTACK_RADIO.off();

      if((clock_time() - discovery_start_time) < (10 * CLOCK_SECOND)) {
        NumSleep = 1; // high duty cycle for determinism in first 10 seconds
      } else {
        NumSleep = random_rand() % (2 * SLEEP_CYCLE + 1);
      }
      printf(" Sleep for %d slots \n", NumSleep);

      for(i = 0; i < NumSleep; i++){
        rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }
  }
  
  PT_END(&pt);
}

// Main thread that handles the neighbour discovery process
PROCESS_THREAD(nbr_discovery_process, ev, data) {
  PROCESS_BEGIN();

  // NEW: Record the start time of discovery
  discovery_start_time = clock_time();

  data_packet.src_id = node_id; 
  data_packet.seq = 0;
  
  nullnet_set_input_callback(receive_packet_callback);
  linkaddr_copy(&dest_addr, &linkaddr_null);

  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending packet of size %d Bytes\n", node_id, (int)sizeof(data_packet_struct));

  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);
  
  PROCESS_END();
}