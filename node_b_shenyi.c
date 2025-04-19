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


#define SAMPLES     20
#define CHUNK_SIZE  20

#define PKT_REQUEST 0x02
#define PKT_DATA    0x03
#define PKT_ACK     0x04

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

// Global variables
static uint8_t peer_set = 0;
static uint8_t chunks_rx = 0;
static struct rtimer timer_rtimer;
static linkaddr_t peer;
static rtimer_clock_t interval = RTIMER_SECOND / 4;
static int16_t light_buf[SAMPLES];
static int16_t motion_buf[SAMPLES];

typedef struct __attribute__((packed)) {
  uint8_t  type;
  uint8_t  seq;
  int16_t  payload[CHUNK_SIZE*2];
} data_pkt_t;

static void send_ack(const linkaddr_t *dest, uint8_t seq) { 
  uint8_t ack[2]={PKT_ACK, seq};
  nullnet_buf=ack;
  nullnet_len=2;
  NETSTACK_NETWORK.output(dest);
} 

/* input */
static void node_b_callback(const void *data, uint16_t len,const linkaddr_t *src,const linkaddr_t *dest){
  if(len<1) return;

  uint8_t type = ((uint8_t*)data)[0];

  if(type == PKT_REQUEST){
    linkaddr_copy(&peer,src);
    peer_set = 1;
    chunks_rx = 0;
    printf("%lu DETECT %u RSSI: %d\n", clock_seconds(), src->u8[7], (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));
    /* respond by expecting data; nothing else to send */

  } else if(type == PKT_DATA && peer_set && linkaddr_cmp(src,&peer)) {
    const data_pkt_t *p=data;

    for(uint8_t i=0;i<CHUNK_SIZE;i++){
      uint8_t idx = p -> seq*CHUNK_SIZE + i;
      light_buf[idx] = p -> payload[2*i];
      motion_buf[idx] = p -> payload[2*i+1];
    }

    send_ack(src, p -> seq);
    chunks_rx++;

    if(chunks_rx * CHUNK_SIZE >= SAMPLES){
      printf("Light:"); for(uint8_t i=0; i < SAMPLES; i++) printf(i?", %d":" %d", light_buf[i]);
      printf("\nMotion:"); for(uint8_t i=0; i < SAMPLES; i++) printf(i?", %d":" %d", motion_buf[i]); printf("\n");
      peer_set=0;
    }
  }
}

/* 
 * timer_callback() is invoked every 250ms.
 * It both polls sensors and advances the state machine.
 */
void timer_callback(struct rtimer *t, void *ptr) {
  
  // Schedule the next callback after 250ms.
  rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);
}

PROCESS_THREAD(process_rtimer, ev, data) {
    PROCESS_BEGIN();
    nullnet_set_input_callback(node_b_callback);
    
    while(1) {
        // Start the periodic callback (every 250 ms)
        rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0, timer_callback, NULL);
        PROCESS_YIELD();
    }

    PROCESS_END();
}