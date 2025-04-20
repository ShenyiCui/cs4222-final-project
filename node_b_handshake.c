#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"

/* ------------ parameters ------------ */
#define SAMPLES     60
#define CHUNK_SIZE  20

#define WAKE_TIME RTIMER_SECOND/10    // 10 HZ, 0.1s
#define SLEEP_CYCLE  9        	      // 0 for never sleep
#define SLEEP_SLOT RTIMER_SECOND/10   // sleep slot should not be too large to prevent overflow
#define NUM_SEND 2

/* ------------ packet types ------------ */
#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02
#define PKT_DATA     0x03
#define PKT_ACK      0x04
#define PKT_REQ_ACK  0x05   /* new */

/* ------------ buffers ------------ */
static uint8_t  chunks_rx = 0;
static int16_t  light_buf[SAMPLES];
static int16_t  motion_buf[SAMPLES];

static struct pt pt; // Protothread variable
// Current time stamp of the node
unsigned long curr_timestamp;
static struct rtimer rt __attribute__((unused));

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;
  int16_t  payload[CHUNK_SIZE*2];
} data_packet_struct;

/* ------------ helpers ------------ */
static void send_ack(const linkaddr_t *dest, uint8_t seq){
  uint8_t ack[2] = { PKT_ACK , seq };
  nullnet_buf = ack;
  nullnet_len = 2;
  NETSTACK_NETWORK.output(dest);
  printf("TX ACK seq %d\n", seq);
}

char listen_beacon(struct rtimer *t, void *ptr) {
  static uint16_t i = 0;
  static int NumSleep = 0;
 
  // Begin the protothread
  PT_BEGIN(&pt);
  // Get the current time stamp
  curr_timestamp = clock_time();
  while(1){

    // radio on
    NETSTACK_RADIO.on();

    for(i = 0; i < NUM_SEND; i++){
      // wait for WAKE_TIME before sending the next packet
      if(i != (NUM_SEND - 1)){
        rtimer_set(t, RTIMER_TIME(t) + WAKE_TIME, 1, (rtimer_callback_t)listen_beacon, ptr);
        PT_YIELD(&pt);
      }
    }

    if(SLEEP_CYCLE != 0){    
      NETSTACK_RADIO.off();
      // SLEEP_SLOT cannot be too large as value will overflow,
      // to have a large sleep interval, sleep many times instead

      // get a value that is uniformly distributed between 0 and 2*SLEEP_CYCLE
      // the average is SLEEP_CYCLE 
      NumSleep = 7;  
      for(i = 0; i < NumSleep; i++){
        rtimer_set(t, RTIMER_TIME(t) + SLEEP_SLOT, 1, (rtimer_callback_t)listen_beacon, ptr);
        PT_YIELD(&pt);
      }

    }
  }
  
  PT_END(&pt);
}

/* ------------ RX callback ------------ */
static void node_b_rx(const void *data, uint16_t len,
                      const linkaddr_t *src, const linkaddr_t *dest){
  if(!len) return;
  uint8_t type = ((const uint8_t*)data)[0];

  printf("%lu RX %02x:%02x type=%d RSSI=%d\n",
         clock_seconds(), src->u8[0], src->u8[1], type,
         (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));

  if(type == PKT_REQUEST) {
    uint8_t ra[2] = { PKT_REQ_ACK , 0x00 };
    nullnet_buf = ra;
    nullnet_len = 2;
    NETSTACK_NETWORK.output(src);
    printf("TX REQ_ACK\n\n");
  } else if(type == PKT_DATA) {
    static data_packet_struct pkt;
    memcpy(&pkt, data, len);

    printf("RX DATA chunk %d\n", pkt.seq);
    for(uint8_t i=0;i<CHUNK_SIZE;i++){
      uint8_t idx = pkt.seq*CHUNK_SIZE + i;
      light_buf[idx]  = pkt.payload[2*i];
      motion_buf[idx] = pkt.payload[2*i+1];
      printf(" sample %d  light=%d  motion=%d\n",
             idx, light_buf[idx], motion_buf[idx]);
    }
    send_ack(src, pkt.seq);
    chunks_rx++;
  }
}

/* ------------ Contiki process ------------ */
PROCESS(node_b_proc,"Node B");
AUTOSTART_PROCESSES(&node_b_proc);

PROCESS_THREAD(node_b_proc, ev, data){
  PROCESS_BEGIN();
  nullnet_set_input_callback(node_b_rx);
  rtimer_set(&rt, RTIMER_NOW() + (RTIMER_SECOND / 1000), 1, (rtimer_callback_t)listen_beacon, NULL);
  PROCESS_END();
}