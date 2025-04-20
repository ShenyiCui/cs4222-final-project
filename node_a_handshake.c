#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "contiki.h"
#include "sys/rtimer.h"
#include "board-peripherals.h"
#include <stdint.h>

#include "net/netstack.h"
#include "net/nullnet/nullnet.h"
#include "net/packetbuf.h"
#include "net/linkaddr.h"
#include <string.h>
#include "node-id.h"

PROCESS(process_rtimer, "RTimer");
AUTOSTART_PROCESSES(&process_rtimer);

/* ------------ parameters ------------ */
#define SAMPLE_INTERVAL    CLOCK_SECOND          // 1 Hz
#define SAMPLES            60
#define CHUNK_SIZE         20                   // 3 chunks
#define SEND_CHUNK_INTERVAL      (RTIMER_SECOND / 4) // 250ms
/* radio duty‑cycling for request/ack phase */
#define WAKE_TIME   (RTIMER_SECOND / 10)   /* 100 ms listening window  */
#define SLEEP_SLOT  (RTIMER_SECOND / 10)   /* 100 ms sleep before next request */

/* ------------ packet types ------------ */
#define PKT_BEACON   0x01
#define PKT_REQUEST  0x02   // request for data
#define PKT_DATA     0x03   // sensor chunk
#define PKT_ACK      0x04   // ack each chunk
#define PKT_REQ_ACK  0x05   // reply to PKT_REQUEST

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

static data_pkt_t data_packet;


/* ------------ link state ------------ */
typedef enum { LINK_SEARCHING = 0, LINK_UP = 1 } link_state_t;
static link_state_t link_state = LINK_SEARCHING;

/* ------------ globals ------------ */
static struct rtimer timer_rtimer;
static rtimer_clock_t interval = RTIMER_SECOND / 4;
static int16_t light_buf[SAMPLES];
static int16_t motion_buf[SAMPLES];
static uint8_t sample_idx = 0;
static int      curr_chunk = 0;

static uint8_t  good_cnt  = 0;           /* consecutive REQ‑ACK ≥ ‑70 dBm */
#define RSSI_GOOD_THRESHOLD (-70)
static int            peer_set = 0;
static linkaddr_t     peer;

static int awaiting_ack   = 0;   /* 1 while waiting for ACK               */
static int last_sent_seq  = -1;  /* sequence number of the last chunk TX  */

/* forward decls */
static void send_chunks(struct rtimer *t, void *ptr);
static void send_request(struct rtimer *t, void *ptr);
static void listen_window_end(struct rtimer *t, void *ptr);
static void chunk_listen_end(struct rtimer *t, void *ptr);

/* ------------ sensor helpers ------------ */
static void   init_opt_reading(void) { SENSORS_ACTIVATE(opt_3001_sensor); }
static void   init_mpu_reading(void){ mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL); }

static int get_light_reading(void){
  int v = opt_3001_sensor.value(0);
  init_opt_reading();
  return (v == CC26XX_SENSOR_READING_ERROR) ? -1 : v/100;
}
static float get_mpu_reading(void){
  int ax = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X)/100;
  int ay = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y)/100;
  int az = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z)/100;
  return sqrtf((float)(ax*ax + ay*ay + az*az));
}

/* ------------ request sender ------------ */
static void send_request(struct rtimer *t, void *ptr){
  if(link_state != LINK_SEARCHING) return;

  req_pkt_t req = { PKT_REQUEST, node_id };
  nullnet_buf   = (uint8_t *)&req;
  nullnet_len   = sizeof(req);
  NETSTACK_NETWORK.output(NULL);          /* broadcast */
  printf("TX REQUEST\n");

  /* Keep the radio on so we can hear the REQ_ACK replies */
  NETSTACK_RADIO.on();

  /* After WAKE_TIME, close the window and schedule the next action */
  rtimer_set(t, RTIMER_NOW() + WAKE_TIME, 0,
             listen_window_end, NULL);
}

/* ------------ listen window end ------------ */
static void listen_window_end(struct rtimer *t, void *ptr){
  /* Close the radio after the listening window */
  NETSTACK_RADIO.off();

  if(link_state == LINK_SEARCHING){
      /* No good link yet – retry after a sleep slot */
      rtimer_set(t, RTIMER_NOW() + SLEEP_SLOT, 0,
                 send_request, NULL);
  } else if(link_state == LINK_UP){
      /* Good link established during this window – start data phase */
      rtimer_set(t, RTIMER_NOW() + SEND_CHUNK_INTERVAL, 0,
                 send_chunks, NULL);
  }
}

/* ------------ sampling timer ------------ */
static void timer_callback(struct rtimer *t, void *ptr){
  light_buf[sample_idx]  = get_light_reading();
  motion_buf[sample_idx] = (int)get_mpu_reading();
  printf("COLLECTING DATA: Sample %u  light=%d  mpu=%d\n",
         sample_idx, light_buf[sample_idx], motion_buf[sample_idx]);
  sample_idx++;

  if(sample_idx < SAMPLES){
    rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0,
               timer_callback, NULL);
  } else {
    /* readings done – start link test */
    curr_chunk  = 0;
    link_state  = LINK_SEARCHING;
    peer_set    = 0;
    good_cnt    = 0;
    rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0,
               send_request, NULL);
  }
}

/* ------------ RX callback ------------ */
static void receive_cb(const void *data, uint16_t len,
                       const linkaddr_t *src, const linkaddr_t *dest){
  if(!len) return;
  uint8_t type = ((const uint8_t*)data)[0];

  if(type == PKT_REQ_ACK) {
    ack_pkt_t *ackp = (ack_pkt_t *)data;
    uint16_t   sender_id = ackp->src_id;

    signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    if(!peer_set){
        linkaddr_copy(&peer, src);
        peer_set = 1;
        good_cnt = 0;
    }
    if(linkaddr_cmp(src,&peer) && rssi >= RSSI_GOOD_THRESHOLD){
        good_cnt++;
    } else if(linkaddr_cmp(src,&peer)){
        good_cnt = 0;
    }

    printf("%lu DETECT node %u  REQ_ACK cnt=%u  rssi=%d\n",
            clock_seconds(), sender_id, good_cnt, rssi);

    if(good_cnt >= 3 && link_state == LINK_SEARCHING){
        link_state = LINK_UP;
        printf("LINK UP: start data transfer\n\n");
        /* first chunk is scheduled by listen_window_end() */
    }
  } else if(type == PKT_ACK) {
    ack_pkt_t *ack = (ack_pkt_t *)data;
    uint8_t ackseq = ack->seq;
    signed short rssi = (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI);

    if(ackseq == last_sent_seq){
      awaiting_ack = 0;          /* ACK heard within listening window */
    }

    if(ackseq == curr_chunk) {
      uint16_t sender_id = ack->src_id;
      printf("%lu DETECT node %u  PKT_ACK seq=%u  rssi=%d\n",
        clock_seconds(), sender_id, curr_chunk, rssi);

      if((curr_chunk + 1)*CHUNK_SIZE >= SAMPLES){
        printf("Transfer complete\n");
        memset(light_buf, 0, sizeof(light_buf));
        memset(motion_buf,0, sizeof(motion_buf));
        sample_idx = 0;
        peer_set   = 0;
        good_cnt   = 0;
        curr_chunk = -1;
      }else{
        curr_chunk++;
      }
    }
  }
}

/* ------------ chunk sender ------------ */
static void send_chunks(struct rtimer *t, void *ptr) {
  if(link_state != LINK_UP || curr_chunk == -1){
    return;                                   /* nothing to do */
  }
  // printf in the format: <timestamp_in_seconds> TRANSFER <nodeID> <avg. link quality>
  printf("%lu TRANSFER-FROM %u RSSI: %d\n", clock_seconds(), node_id, (signed short)packetbuf_attr(PACKETBUF_ATTR_RSSI));

  last_sent_seq  = curr_chunk;
  awaiting_ack   = 1;

  data_packet.type   = PKT_DATA;
  data_packet.src_id = node_id;
  data_packet.seq    = curr_chunk;
  for(uint8_t i=0;i<CHUNK_SIZE;i++){
    uint8_t idx = curr_chunk*CHUNK_SIZE + i;
    data_packet.payload[2*i]     = light_buf[idx];
    data_packet.payload[2*i +1 ] = motion_buf[idx];
  }
  nullnet_buf = (uint8_t*)&data_packet;
  nullnet_len = sizeof(data_packet);
  NETSTACK_NETWORK.output(&peer);

  /* keep the radio on to listen for ACKs */
  NETSTACK_RADIO.on();

  /* close listening window in WAKE_TIME */
  rtimer_set(t, RTIMER_NOW() + WAKE_TIME, 0,
             chunk_listen_end, NULL);
}

/* ------------ end of ACK listening window ------------ */
static void chunk_listen_end(struct rtimer *t, void *ptr){
  NETSTACK_RADIO.off();

  if(link_state != LINK_UP) return;

  if(awaiting_ack){
    /* no ACK – retry same chunk after a short sleep */
    rtimer_set(t, RTIMER_NOW() + SLEEP_SLOT, 0,
               send_chunks, NULL);
  }else if(curr_chunk != -1){
    /* ACK received – next chunk (or finished) */
    rtimer_set(t, RTIMER_NOW() + SEND_CHUNK_INTERVAL, 0,
               send_chunks, NULL);
  }
}

/* ------------ Contiki process ------------ */
PROCESS_THREAD(process_rtimer, ev, data){
  PROCESS_BEGIN();
  init_opt_reading();
  init_mpu_reading();
  nullnet_set_input_callback(receive_cb);

  rtimer_set(&timer_rtimer, RTIMER_NOW() + interval, 0,
             timer_callback, NULL);
  PROCESS_END();
}
