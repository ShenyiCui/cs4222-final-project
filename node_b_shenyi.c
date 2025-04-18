#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "contiki.h"
#include "sys/rtimer.h"
#include "net/nullnet/nullnet.h"
#include "net/netstack.h"
#include "net/packetbuf.h"


#define SAMPLES     60
#define CHUNK_SIZE  20

#define PKT_BEACON  0x01
#define PKT_DATA    0x03
#define PKT_ACK     0x04

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

// Global variables
static uint8_t chunks_rx = 0;
static struct rtimer timer_rtimer;
static rtimer_clock_t interval = RTIMER_SECOND / 4;
static uint8_t beacon_byte = PKT_BEACON;
static int16_t light_buf[SAMPLES];
static int16_t motion_buf[SAMPLES];

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;
  int16_t  payload[CHUNK_SIZE*2];
} data_pkt_t;

typedef struct {
  uint8_t  type;
  uint8_t  seq;
  int16_t  payload[CHUNK_SIZE*2]; // light,motion interleaved
} data_packet_struct;

static void send_ack(const linkaddr_t *dest, uint8_t seq) { 
  uint8_t ack[2]={PKT_ACK, seq};
  nullnet_buf=ack;
  nullnet_len=2;
  NETSTACK_NETWORK.output(dest);
} 

static void send_beacon(void){
  nullnet_buf=&beacon_byte;
  nullnet_len=1;
  NETSTACK_NETWORK.output(NULL);
}

/* input */
static void node_b_callback(const void *data, uint16_t len, const linkaddr_t *src, const linkaddr_t *dest){
  if(len != sizeof(data_packet_struct)) return;
  static data_packet_struct pkt; memcpy(&pkt,data,len);

  // uint8_t type = ((uint8_t*)data)[0];

  // Print Recieved a Packet
  printf("%lu RX %02x:%02x RSSI: %d\n", clock_seconds(), src->u8[0], src->u8[1], (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));

  // printf("Packet Type: %d\n", type);
  
  // if(type == PKT_DATA) {
  printf("Received packet from %u\n", src->u8[7]);
  printf("Received chunk %d\n", pkt.seq);

  for(uint8_t i=0;i<CHUNK_SIZE;i++){
    uint8_t idx = pkt.seq * CHUNK_SIZE + i;
    light_buf[idx] = pkt.payload[2*i];
    motion_buf[idx] = pkt.payload[2*i+1];
  }

  send_ack(src, pkt.seq);
  chunks_rx++;

  if(chunks_rx * CHUNK_SIZE >= SAMPLES){
    printf("Light:"); 
    for(uint8_t i=0; i < SAMPLES; i++) {
      printf(i?", %d":" %d", light_buf[i]);
    }
    printf("\nMotion:");
    for(uint8_t i=0; i < SAMPLES; i++) {
      printf(i?", %d":" %d", motion_buf[i]);
    }
    printf("\n");
  }
  // }
}

/* 
 * timer_callback() is invoked every 250ms.
 * It both polls sensors and advances the state machine.
 */
void timer_callback(struct rtimer *t, void *ptr) {
  send_beacon();
  rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);
}

PROCESS_THREAD(process_rtimer, ev, data) {
    PROCESS_BEGIN();
    nullnet_set_input_callback(node_b_callback);
    // Start the periodic callback (every 250 ms)
    rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);

    PROCESS_END();
}