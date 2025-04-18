// RECEIVER CODE

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "contiki.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"
#include "node-id.h"

#define SAMPLES     60 // No. of samples we are collecting
#define CHUNK_SIZE  20 // Number of readings in each packet (chunk)

#define WAKE_TIME       (RTIMER_SECOND / 10)   // Wake time for neighbour discovery
#define SLEEP_INTERVAL  (RTIMER_SECOND / 4)    // Sleep time between receiving

// Packet types
#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02
#define PKT_DATA     0x03
#define PKT_ACK      0x04
#define PKT_REQ_ACK  0x05

typedef struct __attribute__((packed)) {
  uint8_t  type;  // Packet type        
  uint16_t src_id;   
} req_pkt_t;        

typedef struct __attribute__((packed)) {
  uint8_t  type;         
  uint16_t src_id;      
  uint8_t  seq; // Chunk number          
} ack_pkt_t;      

typedef struct __attribute__((packed)) {
  uint8_t  type;       
  uint16_t src_id;
  uint8_t  seq;         
  int16_t  payload[CHUNK_SIZE * 2]; // Light and motion data
} data_pkt_t;


static uint8_t chunks_received   = 0; // No. of chunks received so far
static uint8_t is_tranmission_complete = 0; 
static int16_t  light_readings[SAMPLES];
static int16_t  motion_readings[SAMPLES];

static struct rtimer rt;
static void end_listen(struct rtimer *t, void *ptr);
static void start_listen(struct rtimer *t, void *ptr);
static void node_b_receive_callback(const void *data, uint16_t len,
                      const linkaddr_t *src, const linkaddr_t *dest);

// End listening and schedule next listen
static void end_listen(struct rtimer *t, void *ptr) {
  NETSTACK_RADIO.off();
  rtimer_set(&rt, RTIMER_NOW() + SLEEP_INTERVAL, 0, start_listen, NULL);
}

// Start listening 
static void start_listen(struct rtimer *t, void *ptr) {
  NETSTACK_RADIO.on();
  rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 0, end_listen, NULL);
}


static void node_b_receive_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest) {
  if(len == 0) {
    return;
  }
  uint8_t packet_type = ((const uint8_t*)data)[0];

  const req_pkt_t *header = (const req_pkt_t *)data;
  printf("%lu DETECT node %u\n", clock_seconds(), header->src_id);

  if(packet_type == PKT_REQUEST) {
    ack_pkt_t ra = { PKT_REQ_ACK, node_id, 0 };
    nullnet_buf = (uint8_t *)&ra;
    nullnet_len = sizeof(ra);
    NETSTACK_NETWORK.output(src);
    printf("TX REQ_ACK\n\n");
  } else if(packet_type == PKT_DATA) {
    data_pkt_t pkt;
    memcpy(&pkt, data, len);

    printf("RX DATA chunk %d\n", pkt.seq);
    
    for(uint8_t i=0; i<CHUNK_SIZE; i++) {
      uint8_t idx = pkt.seq*CHUNK_SIZE + i;
      light_readings[idx]  = pkt.payload[2*i];
      motion_readings[idx] = pkt.payload[2*i+1];
      // printf(" sample %d  light=%d  motion=%d\n",
      //        idx, light_readings[idx], motion_readings[idx]);
    }

    chunks_received++;
    if(chunks_received == 3) {
        is_tranmission_complete = 1;
    }

    ack_pkt_t ack = { PKT_ACK, node_id, pkt.seq };
    for(int i = 0;i < 5;i++){
        nullnet_buf = (uint8_t *)&ack;
        nullnet_len = sizeof(ack);
        NETSTACK_NETWORK.output(src);
    }
    printf("Transmitted ACK for chunk %d\n\n", pkt.seq);
  }

  if(is_tranmission_complete) {
    printf("Light:");
    for(int i=0;i<SAMPLES;i++){
        printf(" %d%s", light_readings[i], i == SAMPLES-1 ? "\n" : " ,");
    }
    printf("Motion:");
    for(int i=0;i<SAMPLES;i++){
        printf(" %d%s", motion_readings[i], i == SAMPLES-1 ? "\n" : " ,");
    }

    // Reset the state
    chunks_received   = 0;
    is_tranmission_complete = 0;
  }
}

PROCESS(node_b_proc,"Node B");
AUTOSTART_PROCESSES(&node_b_proc);

PROCESS_THREAD(node_b_proc, ev, data){
  PROCESS_BEGIN();
  nullnet_set_input_callback(node_b_receive_callback);
  rtimer_set(&rt, RTIMER_NOW() + WAKE_TIME, 0, start_listen, NULL);
  PROCESS_END();
}