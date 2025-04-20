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

/* ------------ parameters ------------ */
#define SAMPLES     60
#define CHUNK_SIZE  20

#define WAKE_TIME       (RTIMER_SECOND / 10)   /* 100 ms listen window */
#define SLEEP_INTERVAL  (RTIMER_SECOND / 4)    /* 250 ms sleep */
#define SLEEP_CYCLE  9        	      // 0 for never sleep
#define SLEEP_SLOT RTIMER_SECOND/10   // sleep slot should not be too large to prevent overflow
#define NUM_SEND 2

/* ------------ packet types ------------ */
#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02
#define PKT_DATA     0x03
#define PKT_ACK      0x04
#define PKT_REQ_ACK  0x05   /* new */

/* ---- common packet formats ---- */
typedef struct __attribute__((packed)) {
  uint8_t  type;          /* PKT_* */
  uint16_t src_id;        /* node_id of sender                */
} req_pkt_t;              /* also reused for plain beacons     */

typedef struct __attribute__((packed)) {
  uint8_t  type;          /* PKT_* */
  uint16_t src_id;        /* node_id of sender                */
  uint8_t  seq;           /* chunk number (or 0)              */
} ack_pkt_t;              /* used for REQ_ACK and DATA_ACK    */

typedef struct __attribute__((packed)) {
  uint8_t  type;          /* PKT_DATA                         */
  uint16_t src_id;        /* node_id of sender                */
  uint8_t  seq;           /* chunk number                     */
  int16_t  payload[CHUNK_SIZE * 2];
} data_pkt_t;

/* ------------ buffers ------------ */
static uint8_t chunks_rx   = 0;      /* how many chunks so far   */
static uint8_t expect_seen = 0;      /* flag: 1 after all 3 rx’d */
static int16_t  light_buf[SAMPLES];
static int16_t  motion_buf[SAMPLES];

static struct rtimer listen_timer;
static void end_listen(struct rtimer *t, void *ptr);
static void start_listen(struct rtimer *t, void *ptr);
static void node_b_rx(const void *data, uint16_t len,
                      const linkaddr_t *src, const linkaddr_t *dest);

/* ------------ helpers ------------ */
/* Turn radio off, then sleep, then restart listening */
static void end_listen(struct rtimer *t, void *ptr) {
  NETSTACK_RADIO.off();
  rtimer_set(&listen_timer,
             RTIMER_NOW() + SLEEP_INTERVAL,
             0,
             (rtimer_callback_t)start_listen,
             NULL);
}

/* Turn radio on and schedule it off after WAKE_TIME */
static void start_listen(struct rtimer *t, void *ptr) {
  NETSTACK_RADIO.on();
  rtimer_set(&listen_timer,
             RTIMER_NOW() + WAKE_TIME,
             0,
             (rtimer_callback_t)end_listen,
             NULL);
}

/* ------------ RX callback ------------ */
static void node_b_rx(const void *data, uint16_t len,
                      const linkaddr_t *src, const linkaddr_t *dest){
  if(!len) return;
  uint8_t type = ((const uint8_t*)data)[0];

  const req_pkt_t *header = (const req_pkt_t *)data;
  printf("%lu DETECT node %u\n", clock_seconds(), header->src_id);

  if(type == PKT_REQUEST) {
    ack_pkt_t ra = { PKT_REQ_ACK, node_id, 0 };
    nullnet_buf = (uint8_t *)&ra;
    nullnet_len = sizeof(ra);
    NETSTACK_NETWORK.output(src);
    printf("TX REQ_ACK\n\n");
  } else if(type == PKT_DATA) {
    data_pkt_t pkt;
    memcpy(&pkt, data, len);

    printf("RX DATA chunk %d\n", pkt.seq);
    
    for(uint8_t i=0; i<CHUNK_SIZE; i++) {
      uint8_t idx = pkt.seq*CHUNK_SIZE + i;
      light_buf[idx]  = pkt.payload[2*i];
      motion_buf[idx] = pkt.payload[2*i+1];
      // printf(" sample %d  light=%d  motion=%d\n",
      //        idx, light_buf[idx], motion_buf[idx]);
    }

    /* mark that one more chunk arrived */
    chunks_rx++;
    if(chunks_rx == 3) {
        expect_seen = 1;
    }

    ack_pkt_t ack = { PKT_ACK, node_id, pkt.seq };
    for(int i=0;i<5;i++){
        nullnet_buf = (uint8_t *)&ack;
        nullnet_len = sizeof(ack);
        NETSTACK_NETWORK.output(src);
    }
    printf("TX DATA_ACK x5 seq %d\n\n", pkt.seq);
  }

  if(expect_seen) {
    printf("Light:");
    for(int i=0;i<SAMPLES;i++){
        printf(" %d%s", light_buf[i], (i==SAMPLES-1)?"":" ,");
    }
    printf("\nMotion:");
    for(int i=0;i<SAMPLES;i++){
        printf(" %d%s", motion_buf[i], (i==SAMPLES-1)?"":" ,");
    }
    printf("\n");

    /* reset for next transfer */
    chunks_rx   = 0;
    expect_seen = 0;
  }
}

/* ------------ Contiki process ------------ */
PROCESS(node_b_proc,"Node B");
AUTOSTART_PROCESSES(&node_b_proc);

PROCESS_THREAD(node_b_proc, ev, data){
  PROCESS_BEGIN();
  nullnet_set_input_callback(node_b_rx);
  rtimer_set(&listen_timer,
              RTIMER_NOW() + WAKE_TIME,
              0,
              (rtimer_callback_t)start_listen,
              NULL);
  PROCESS_END();
}