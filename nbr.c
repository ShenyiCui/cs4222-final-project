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
#define WAKE_TIME RTIMER_SECOND / 10
#define SLEEP_SLOT RTIMER_SECOND / 10
#define FLAG_ACK 0x01

// broadcast address
linkaddr_t dest_addr;
#define NUM_SEND 2

#define LOW_SLEEP_COUNT 18  // Adjust as needed for power saving

#define MODE_NORMAL 0   // low duty cycle 
#define MODE_AGGRESSIVE 1   // send beacons aggressively until ACK seen or 10s passes
#define MODE_ACK 2  
#define MODE_COMPLETE 3  


typedef struct {
  unsigned long src_id;
  unsigned long timestamp;
  unsigned long seq;
  uint8_t phase;
  uint8_t flags; 
} data_packet_struct;


static struct rtimer rt __attribute__((unused));
static struct pt pt;
static data_packet_struct data_packet;
unsigned long curr_timestamp;

static uint8_t mode = 0;
static unsigned long aggressive_start_time = 0;
static unsigned long ack_start_time = 0;  
static uint8_t ack_started = 0; 

PROCESS(nbr_discovery_process, "cc2650 neighbour discovery process");

void receive_packet_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if(len != sizeof(data_packet_struct)) return;
  static data_packet_struct pkt; memcpy(&pkt,data,len);

  printf("RX seq %lu from %lu phase %u flags 0x%02X\n", pkt.seq, pkt.src_id, pkt.phase, pkt.flags);

  switch(mode) {
  case MODE_NORMAL:
      // detect other device -> go aggressive
      mode = MODE_AGGRESSIVE;
      aggressive_start_time = clock_time();
      printf("MODE_NORMAL -> MODE_AGGRESSIVE\n");
      break;
  case MODE_AGGRESSIVE:
      if(pkt.flags & FLAG_ACK) {
          mode = MODE_COMPLETE;
          printf("ACK seen -> MODE_COMPLETE\n");
      } else if(!ack_started) {
          // peer still aggressive - become ACK sender
          mode = MODE_ACK;
          ack_start_time = clock_time();
          ack_started = 1;
          printf("Start ACK window\n");
      }
      break;
  case MODE_ACK:
      if(pkt.flags & FLAG_ACK) {
          mode = MODE_COMPLETE;
          printf("Peer ACK -> MODE_COMPLETE\n");
      }
      break;
  default:
      break;
  }
}

char sender_scheduler(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int sleep_count = 0;
  unsigned long current;

  PT_BEGIN(&pt);

  curr_timestamp = clock_time();
  printf("Start clock %lu ticks, timestamp %3lu.%03lu\n", curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND);

  while(1) {
    if(mode == MODE_COMPLETE) {
      printf("Discovery complete, stopping transmissions and entering sleep mode.\n");
      NETSTACK_RADIO.off();
      PT_EXIT(&pt);
    }
    
    NETSTACK_RADIO.on();
    
    if(mode == MODE_ACK) {
      data_packet.flags = FLAG_ACK;
      data_packet.phase = MODE_AGGRESSIVE;
    } else {
      data_packet.flags = 0;
      data_packet.phase = (mode==MODE_AGGRESSIVE)? MODE_AGGRESSIVE : MODE_NORMAL;
    }
    
    for(i = 0; i < NUM_SEND; i++) {
      nullnet_buf = (uint8_t *)&data_packet;
      nullnet_len = sizeof(data_packet);
      
      data_packet.seq++;
      curr_timestamp = clock_time();
      data_packet.timestamp = curr_timestamp;
      
      printf("Send seq# %lu  @ %8lu ticks   %3lu.%03lu, phase %d\n", data_packet.seq, curr_timestamp, curr_timestamp / CLOCK_SECOND, ((curr_timestamp % CLOCK_SECOND) * 1000) / CLOCK_SECOND, data_packet.phase);
      
      NETSTACK_NETWORK.output(&dest_addr);
      
      if(i != (NUM_SEND - 1)) {
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, sender_scheduler, ptr);
        PT_YIELD(&pt);
      }
    }
    
    NETSTACK_RADIO.off();
    
    current = clock_time();
    
    if(mode == MODE_AGGRESSIVE) {
      sleep_count = 1;
      if(current - aggressive_start_time >= 10 * CLOCK_SECOND) {
        mode = MODE_NORMAL;
        printf("10s aggressive mode timeout -> MODE_NORMAL\n");
      }
    } else if(mode == MODE_ACK) {
      sleep_count = 1;
      if(current - ack_start_time >= 2 * CLOCK_SECOND) {
        mode = MODE_COMPLETE;
        ack_started = 0;
        printf("ACK window done -> MODE_COMPLETE\n");
      }
    } else if(mode == MODE_NORMAL) {
      sleep_count = LOW_SLEEP_COUNT;
    }
    
    printf("Sleep for %d slots (mode %d)\n", sleep_count, mode);
    for(i = 0; i < sleep_count; i++){
      rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1,
                 (rtimer_callback_t)sender_scheduler, ptr);
      PT_YIELD(&pt);
    }
  }
  
  PT_END(&pt);
}

__attribute__((used))
PROCESS_THREAD(nbr_discovery_process, ev, data) {
  PROCESS_BEGIN();
  
  data_packet.src_id = node_id;
  data_packet.seq = 0;
  data_packet.phase = 0;  // Start in low duty cycle mode (phase 0).
  
  nullnet_set_input_callback(receive_packet_callback);
  linkaddr_copy(&dest_addr, &linkaddr_null);
  
  printf("CC2650 neighbour discovery\n");
  printf("Node %d will be sending packet of size %d Bytes\n", node_id, (int)sizeof(data_packet_struct));
  
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)sender_scheduler, NULL);
  
  PROCESS_END();
}

AUTOSTART_PROCESSES(&nbr_discovery_process);
